from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Dict, Optional
import json
import asyncio
import traceback
from interfaces.srv import RequestStream, SetAnswer, AddIceCandidate, RemovePeer
from interfaces.msg import WebRTCMessage
from rclpy.callback_groups import ReentrantCallbackGroup
from pathlib import Path
import rclpy
from rclpy.task import Future
from concurrent.futures import ThreadPoolExecutor
import functools
import requests

router = APIRouter()

def get_node_manager():
    from communication_server.communication_server import node_manager
    return node_manager

def load_camera_config():
    """Load camera configuration from ROS2 node via service call"""
    node_manager = get_node_manager()
    node = node_manager.node
    
    try:
        # First try to get camera servers through ROS2
        camera_servers = {}
        for camera_id, camera_info in node.camera_client.camera_servers.items():
            camera_servers[camera_id] = {
                "device_info": camera_info.get("device_info", {}),
                "streams": camera_info.get("streams", {})
            }
        
        # Then get the CAMERAS dictionary
        cameras = node.camera_client.config.get("CAMERAS", {})
        
        return {
            "CAMERAS": cameras,
            "CAMERA_SERVERS": camera_servers
        }
    except Exception as e:
        node.get_logger().error(f"Error loading camera config: {e}")
        
        # Fallback to config file if ROS2 approach fails
        current_path = Path(__file__).resolve()
        current_path_str = str(current_path)
        ros2_ws_index = current_path_str.rfind("ros2_ws/")
        if ros2_ws_index == -1:
            raise FileNotFoundError("ros2_ws directory not found in the path.")
        
        base_path = current_path_str[:ros2_ws_index + len("ros2_ws/")]
        config_path = Path(base_path).parent / 'cams' / 'cameras.json'
        
        if not config_path.exists():
            raise FileNotFoundError(f"Configuration file not found: {config_path}")
        
        with open(config_path, 'r') as f:
            return json.load(f)

class WebRTCConnection:
    def __init__(self, websocket: WebSocket, peer_id: str):
        self.websocket = websocket
        self.peer_id = peer_id
        self.active_cameras = set()

async def call_ros_service(node, service_type, service_name, request):
    cli = node.create_client(service_type, service_name)
    try:
        while not cli.wait_for_service(timeout_sec=1.0):
            node.get_logger().info(f'Waiting for service {service_name}...')
        
        future = cli.call_async(request)
        
        # Create a new event loop for the service call
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        try:
            # Wait for the service call to complete
            while not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                await asyncio.sleep(0.1)
            
            return future.result()
        finally:
            loop.close()
    finally:
        node.destroy_client(cli)

async def handle_webrtc_message(websocket: WebSocket, connection: WebRTCConnection, message: dict):
    node_manager = get_node_manager()
    node = node_manager.node
    
    try:
        print(f"Handling WebRTC message type: {message['type']}")
        
        if message['type'] == 'request-stream':
            camera_id = message['camera_id']
            codec = message.get('codec', 'vp8')
            
            print(f"Creating stream request for camera {camera_id} with codec {codec}")
            
            request = RequestStream.Request()
            request.camera_id = camera_id
            request.peer_id = connection.peer_id
            request.codec = codec
            
            print("Calling ROS service '/camera/request_stream'")
            response = await call_ros_service(
                node, 
                RequestStream, 
                '/camera/request_stream',
                request
            )
            print(f"Stream request response: {response}")
            
            if response.success:
                connection.active_cameras.add(camera_id)
                message = {
                    'type': 'offer',
                    'camera_id': camera_id,
                    'sdp': response.offer_sdp
                }
                print(f"Sending offer to client: {message}")
                await websocket.send_text(json.dumps(message))
            else:
                error_message = {
                    'type': 'error',
                    'camera_id': camera_id,
                    'message': response.error_message
                }
                print(f"Sending error to client: {error_message}")
                await websocket.send_text(json.dumps(error_message))
        
        elif message['type'] == 'answer':
            print(f"Received answer for camera {message['camera_id']}")
            request = SetAnswer.Request()
            request.camera_id = message['camera_id']
            request.peer_id = connection.peer_id
            request.answer_sdp = message['sdp']
            
            response = await call_ros_service(
                node,
                SetAnswer,
                '/camera/set_answer',
                request
            )
            print(f"Set answer response: {response}")
            
            if not response.success:
                await websocket.send_text(json.dumps({
                    'type': 'error',
                    'camera_id': message['camera_id'],
                    'message': response.error_message
                }))
        
        elif message['type'] == 'ice-candidate':
            print(f"Received ICE candidate for camera {message['camera_id']}")
            request = AddIceCandidate.Request()
            request.camera_id = message['camera_id']
            request.candidate_sdp = message['candidate']['candidate']
            request.candidate_sdp_mline_index = message['candidate']['sdpMLineIndex']
            
            response = await call_ros_service(
                node,
                AddIceCandidate,
                '/camera/add_ice_candidate',
                request
            )
            print(f"Add ICE candidate response: {response}")
            
            if not response.success:
                await websocket.send_text(json.dumps({
                    'type': 'error',
                    'camera_id': message['camera_id'],
                    'message': response.error_message
                }))

    except Exception as e:
        print(f"Error handling WebRTC message: {str(e)}")
        traceback.print_exc()
        await websocket.send_text(json.dumps({
            'type': 'error',
            'message': str(e)
        }))

@router.get("/camera-status")
async def get_camera_status():
    """Get status of all camera servers"""
    try:
        camera_config = load_camera_config()
        return camera_config
    except Exception as e:
        return {"error": str(e)}

@router.websocket("/ws/camera-webrtc")
async def webrtc_endpoint(websocket: WebSocket):
    await websocket.accept()
    
    # Generate unique peer ID
    peer_id = str(id(websocket))
    connection = WebRTCConnection(websocket, peer_id)
    
    node_manager = get_node_manager()
    node = node_manager.node
    
    # Load camera config and send to client
    try:
        camera_config = load_camera_config()
        await websocket.send_text(json.dumps({
            'type': 'camera-config',
            'config': camera_config
        }))
    except Exception as e:
        await websocket.send_text(json.dumps({
            'type': 'error',
            'message': f"Error loading camera configuration: {str(e)}"
        }))
    
    # Subscribe to WebRTC messages for this peer
    def webrtc_message_callback(msg):
        if msg.peer_id == peer_id:
            asyncio.create_task(websocket.send_text(json.dumps({
                'type': msg.type,
                'camera_id': msg.camera_id,
                'data': json.loads(msg.data)
            })))
    
    subscriber = node.create_subscription(
        WebRTCMessage,
        '/camera/webrtc_messages',
        webrtc_message_callback,
        10
    )
    
    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            await handle_webrtc_message(websocket, connection, message)
            
    except WebSocketDisconnect:
        # Clean up all active camera connections
        for camera_id in connection.active_cameras:
            request = RemovePeer.Request()
            request.camera_id = camera_id
            request.peer_id = peer_id
            
            # Use a separate thread to call the service
            loop = asyncio.get_event_loop()
            await loop.run_in_executor(
                None,
                lambda: node.call_service('/camera/remove_peer', request)
            )
            
        node.destroy_subscription(subscriber)
        
    except Exception as e:
        node.get_logger().error(f"Error in WebRTC endpoint: {str(e)}")
        # Clean up all active camera connections
        for camera_id in connection.active_cameras:
            request = RemovePeer.Request()
            request.camera_id = camera_id
            request.peer_id = peer_id
            
            try:
                # Use a separate thread to call the service
                loop = asyncio.get_event_loop()
                await loop.run_in_executor(
                    None,
                    lambda: node.call_service('/camera/remove_peer', request)
                )
            except:
                pass
                
        node.destroy_subscription(subscriber)