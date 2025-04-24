import rclpy
from rclpy.node import Node
from interfaces.srv import RequestStream, SetAnswer, AddIceCandidate, RemovePeer
from interfaces.msg import WebRTCMessage
import json
import requests
import subprocess
import time
import threading
import gi
import os
from pathlib import Path
import asyncio
from typing import Dict, Optional
from concurrent.futures import ThreadPoolExecutor
import uuid

# GStreamer imports
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class CameraStream:
    def __init__(self, camera_id: str, rtsp_url: str):
        self.camera_id = camera_id
        self.rtsp_url = rtsp_url
        self.pipeline = None
        self.webrtc = None
        self.peer_connections = {}
        self.pipeline_ready = False
        self.error_message = None
        
        # Initialize GStreamer if needed
        if not Gst.is_initialized():
            Gst.init(None)
    
    def create_pipeline(self):
        """Create and set up the GStreamer pipeline to receive RTSP stream and offer WebRTC"""
        try:
            # Pipeline to receive RTSP and prepare for WebRTC
            pipeline_str = (
                f'rtspsrc location={self.rtsp_url} latency=0 ! '
                'rtph264depay ! h264parse ! '
                'avdec_h264 ! videoconvert ! videoscale ! '
                'video/x-raw,format=I420,width=640,height=480 ! '
                'tee name=t ! '
                'queue ! vp8enc deadline=1 cpu-used=4 target-bitrate=2000000 ! '
                'rtpvp8pay ! '
                'webrtcbin name=webrtc bundle-policy=max-bundle stun-server=stun://stun.l.google.com:19302'
            )
            
            print(f"Creating pipeline with string: {pipeline_str}")
            
            # Create pipeline from string
            self.pipeline = Gst.parse_launch(pipeline_str)
            if not self.pipeline:
                raise Exception("Failed to create pipeline")

            # Get webrtcbin by name
            self.webrtc = self.pipeline.get_by_name('webrtc')
            if not self.webrtc:
                raise Exception("Failed to get webrtcbin element")

            # Connect webrtc signals
            self.webrtc.connect('on-negotiation-needed', self.on_negotiation_needed)
            self.webrtc.connect('on-ice-candidate', self.on_ice_candidate)

            # Set up bus monitoring
            bus = self.pipeline.get_bus()
            bus.add_signal_watch()
            bus.connect('message', self.on_bus_message)

            # Start the pipeline
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                raise Exception("Failed to set pipeline to PLAYING state")

            print(f"Pipeline successfully created and started for {self.rtsp_url}")
            self.pipeline_ready = True
            
        except Exception as e:
            self.error_message = str(e)
            print(f"Error creating pipeline: {e}")
            self.cleanup()
            raise
    
    def on_bus_message(self, bus, message):
        """Handle pipeline bus messages"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Pipeline Error: {err.message}")
            print(f"Debug details: {debug}")
            self.error_message = err.message
            self.pipeline_ready = False
            self.cleanup()
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"Pipeline Warning: {err.message}")
        elif t == Gst.MessageType.STATE_CHANGED:
            if message.src == self.pipeline:
                old_state, new_state, pending_state = message.parse_state_changed()
                print(f"Pipeline state changed from {old_state.value_nick} to {new_state.value_nick}")
                
                # If entering PLAYING state, mark pipeline as ready
                if new_state == Gst.State.PLAYING:
                    self.pipeline_ready = True
    
    def on_negotiation_needed(self, element):
        if not self.pipeline_ready:
            print("Pipeline not ready, skipping negotiation")
            return
            
        print("Negotiation needed, creating offer...")
        try:
            promise = Gst.Promise.new_with_change_func(self.on_offer_created, element)
            element.emit('create-offer', None, promise)
        except Exception as e:
            print(f"Error in negotiation: {e}")
    
    def on_offer_created(self, promise, element):
        print("Offer created, setting local description...")
        try:
            promise.wait()
            reply = promise.get_reply()
            offer = reply.get_value('offer')
            if not offer:
                raise Exception("No offer in promise reply")
            
            sdp = offer.sdp.as_text()
            print(f"Generated SDP:\n{sdp}")
            
            element.emit('set-local-description', offer, None)
            self.pending_offer = {
                'type': 'offer',
                'sdp': sdp
            }
            print("Local description set")
        except Exception as e:
            print(f"Error creating offer: {e}")
            self.error_message = str(e)
            self.cleanup()
    
    def on_ice_candidate(self, webrtc, mlineindex, candidate):
        if not hasattr(self, 'pending_ice_candidates'):
            self.pending_ice_candidates = []
        self.pending_ice_candidates.append({
            'candidate': candidate,
            'sdpMLineIndex': mlineindex
        })
        print(f"ICE candidate collected: {candidate}")

    def cleanup(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
        self.webrtc = None
        self.pipeline_ready = False


class CameraClient(Node):
    def __init__(self):
        super().__init__('camera_client')
        
        # Initialize GStreamer
        Gst.init(None)
        
        # Load camera configuration
        self.config = self._load_config()
        
        # Dictionary to store active camera streams
        self.cameras: Dict[str, CameraStream] = {}
        
        # Dictionary to store camera server information
        self.camera_servers = {}
        
        # GLib main loop for GStreamer
        self.loop = GLib.MainLoop()
        self.loop_thread = threading.Thread(target=self.loop.run)
        self.loop_thread.daemon = True
        self.loop_thread.start()
        
        # Create services
        self.create_services()
        
        # WebRTC message publisher
        self.webrtc_pub = self.create_publisher(
            WebRTCMessage,
            '/camera/webrtc_messages',
            10
        )
        
        # Create scanner timer
        self.scanner_timer = self.create_timer(10.0, self.scan_camera_servers)
        
        self.get_logger().info('Camera Client initialized')
        
        # Initial scan
        self.scan_camera_servers()

    def _load_config(self) -> dict:
        """Load camera configuration and set defaults"""
        current_path = Path(__file__).resolve()
        current_path_str = str(current_path)
        ros2_ws_index = current_path_str.rfind("ros2_ws/")
        if ros2_ws_index == -1:
            config_path = Path(os.path.expanduser("~/.config/camera_client/config.json"))
        else:
            base_path = current_path_str[:ros2_ws_index + len("ros2_ws/")]
            config_path = Path(base_path).parent / 'cams' / 'cameras.json'
        
        default_config = {
            "BASE_PORT": 50000,
            "CAMERAS": {},
            "CAMERA_SERVERS": [
                # Default list of camera servers to scan
                "http://raspberrypi.local:8080/status",
                "http://raspberrypi-2.local:8080/status"
            ]
        }
        
        if config_path.exists():
            try:
                with open(config_path, 'r') as f:
                    config = json.load(f)
                    # Merge with defaults for missing keys
                    if "CAMERA_SERVERS" not in config:
                        config["CAMERA_SERVERS"] = default_config["CAMERA_SERVERS"]
                    return config
            except Exception as e:
                self.get_logger().error(f"Error loading configuration: {e}")
        
        os.makedirs(config_path.parent, exist_ok=True)
        with open(config_path, 'w') as f:
            json.dump(default_config, f, indent=2)
        
        return default_config

    def scan_camera_servers(self):
        """Scan all configured camera servers to discover cameras"""
        self.get_logger().info('Scanning for camera servers...')
        
        for server_url in self.config["CAMERA_SERVERS"]:
            try:
                response = requests.get(server_url, timeout=3)
                if response.status_code == 200:
                    server_info = response.json()
                    self.get_logger().info(f'Found camera server: {server_info["camera_id"]}')
                    
                    # Store/update server information
                    self.camera_servers[server_info["camera_id"]] = server_info
                    
                    # Register streams in our CAMERAS config
                    if "streams" in server_info:
                        for camera_id, stream_info in server_info["streams"].items():
                            full_camera_id = f"{server_info['camera_id']}_{camera_id}"
                            self.config["CAMERAS"][full_camera_id] = stream_info["rtsp_url"]
                            self.get_logger().info(f'Registered camera: {full_camera_id} -> {stream_info["rtsp_url"]}')
            except requests.RequestException as e:
                self.get_logger().warning(f'Failed to connect to camera server {server_url}: {e}')
    
    def create_services(self):
        """Create ROS2 services"""
        self.srv_request_stream = self.create_service(
            RequestStream,
            '/camera/request_stream',
            self._handle_stream_request
        )
        
        self.srv_set_answer = self.create_service(
            SetAnswer,
            '/camera/set_answer',
            self._handle_set_answer
        )
        
        self.srv_add_ice = self.create_service(
            AddIceCandidate,
            '/camera/add_ice_candidate',
            self._handle_add_ice_candidate
        )
        
        self.srv_remove_peer = self.create_service(
            RemovePeer,
            '/camera/remove_peer',
            self._handle_remove_peer
        )

    def _handle_stream_request(self, request, response):
        """Handle incoming stream requests"""
        self.get_logger().info(f"Received stream request: {request}")
        try:
            camera_id = request.camera_id
            peer_id = request.peer_id
            codec = request.codec.lower()
            
            if camera_id not in self.config["CAMERAS"]:
                response.success = False
                response.error_message = f"Camera {camera_id} not found"
                return response
            
            # Get RTSP URL from config
            rtsp_url = self.config["CAMERAS"][camera_id]
            
            # Get or create camera stream
            if camera_id not in self.cameras:
                camera_stream = CameraStream(camera_id, rtsp_url)
                try:
                    camera_stream.create_pipeline()
                except Exception as e:
                    response.success = False
                    response.error_message = str(e)
                    return response
                
                self.cameras[camera_id] = camera_stream
            
            camera_stream = self.cameras[camera_id]
            
            # Check if pipeline is ready
            if not camera_stream.pipeline_ready:
                response.success = False
                response.error_message = camera_stream.error_message or "Pipeline not ready"
                return response
            
            # Wait for offer with timeout
            start_time = self.get_clock().now()
            timeout = 5.0  # 5 seconds timeout
            
            while not hasattr(camera_stream, 'pending_offer'):
                if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                    response.success = False
                    response.error_message = "Timeout waiting for WebRTC offer"
                    return response
                    
                if not camera_stream.pipeline_ready:
                    response.success = False
                    response.error_message = camera_stream.error_message or "Pipeline failed during offer creation"
                    return response
                    
                self.get_logger().info(f"Waiting for offer from camera {camera_id}")
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # Get offer
            offer = camera_stream.pending_offer
            delattr(camera_stream, 'pending_offer')
            
            response.success = True
            response.offer_sdp = offer['sdp']
            response.offer_type = offer['type']
            
            # Store peer connection association
            camera_stream.peer_connections[peer_id] = True
            
        except Exception as e:
            self.get_logger().error(f'Error handling stream request: {str(e)}')
            response.success = False
            response.error_message = str(e)
        
        return response

    def _handle_set_answer(self, request, response):
        """Handle incoming answers"""
        self.get_logger().info(f"Received answer: {request}")
        try:
            camera_id = request.camera_id
            peer_id = request.peer_id
            
            if camera_id not in self.cameras:
                response.success = False
                response.error_message = f"Camera {camera_id} not found"
                return response
            
            camera_stream = self.cameras[camera_id]
            
            # Set remote description (adapted for GStreamer-only implementation)
            from gi.repository import GstSdp, GstWebRTC
            answer = GstWebRTC.WebRTCSessionDescription.new(
                GstWebRTC.WebRTCSDPType.ANSWER,
                GstSdp.SDPMessage.parse_buffer(request.answer_sdp.encode())
            )
            promise = Gst.Promise.new()
            camera_stream.webrtc.emit('set-remote-description', answer, promise)
            promise.wait()
            
            # Send any pending ICE candidates
            if hasattr(camera_stream, 'pending_ice_candidates'):
                for candidate in camera_stream.pending_ice_candidates:
                    self._publish_webrtc_message(
                        peer_id,
                        camera_id,
                        'ice-candidate',
                        candidate
                    )
                delattr(camera_stream, 'pending_ice_candidates')
            
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f'Error handling set answer: {str(e)}')
            response.success = False
            response.error_message = str(e)
        
        return response

    def _handle_add_ice_candidate(self, request, response):
        """Handle incoming ICE candidates"""
        self.get_logger().info(f"Received ICE candidate: {request}")
        try:
            camera_id = request.camera_id
            
            if camera_id not in self.cameras:
                response.success = False
                response.error_message = f"Camera {camera_id} not found"
                return response
            
            camera_stream = self.cameras[camera_id]
            
            # Add ICE candidate
            camera_stream.webrtc.emit(
                'add-ice-candidate',
                request.candidate_sdp_mline_index,
                request.candidate_sdp
            )
            
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f'Error handling ICE candidate: {str(e)}')
            response.success = False
            response.error_message = str(e)
        
        return response

    def _handle_remove_peer(self, request, response):
        """Handle peer removal requests"""
        self.get_logger().info(f"Removing peer: {request}")
        try:
            camera_id = request.camera_id
            peer_id = request.peer_id
            
            if camera_id in self.cameras:
                camera_stream = self.cameras[camera_id]
                
                # Remove peer from camera stream
                if peer_id in camera_stream.peer_connections:
                    del camera_stream.peer_connections[peer_id]
                
                # If no more peers, cleanup camera stream
                if not camera_stream.peer_connections:
                    camera_stream.cleanup()
                    del self.cameras[camera_id]
            
            response.success = True
            
        except Exception as e:
            self.get_logger().error(f'Error handling peer removal: {str(e)}')
            response.success = False
            response.error_message = str(e)
        
        return response

    def _publish_webrtc_message(self, peer_id: str, camera_id: str, message_type: str, data: dict):
        """Publish WebRTC message to the client"""
        self.get_logger().info(f"Publishing WebRTC message: {message_type}")
        msg = WebRTCMessage()
        msg.peer_id = peer_id
        msg.camera_id = camera_id
        msg.type = message_type
        msg.data = json.dumps(data)
        self.webrtc_pub.publish(msg)

    def cleanup(self):
        """Cleanup all camera streams"""
        for camera_stream in self.cameras.values():
            camera_stream.cleanup()
        self.cameras.clear()
        
        if hasattr(self, 'loop') and self.loop.is_running():
            self.loop.quit()
        
        if hasattr(self, 'loop_thread') and self.loop_thread.is_alive():
            self.loop_thread.join(timeout=1.0)

def main(args=None):
    rclpy.init(args=args)
    camera_client = CameraClient()
    try:
        rclpy.spin(camera_client)
    finally:
        camera_client.cleanup()
        camera_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()