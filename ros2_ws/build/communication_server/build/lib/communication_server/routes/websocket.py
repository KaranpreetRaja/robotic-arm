from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from std_msgs.msg import String
import json

router = APIRouter()

def get_node_manager():
    from communication_server.communication_server import node_manager
    return node_manager

@router.websocket("/ws_publish")
async def websocket_endpoint(websocket: WebSocket):
    node_manager = get_node_manager()
    await websocket.accept()
    node_manager.node.websocket_client = websocket
    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            topic = message["topic"]
            message_content = message["message"]
            
            if topic and message_content: 
                if not any(publisher.topic_name == topic for publisher in node_manager.node._publishers):
                    node_manager.node.add_publisher(topic, String)
                
                node_manager.node.publish_message(topic, message_content)

    except WebSocketDisconnect:
        node_manager.node.websocket_client = None
        
@router.websocket("/ws_sub")
async def websocket_subscribe(websocket: WebSocket):
    node = get_node_manager().node
    await websocket.accept()
    node.websocket_client = websocket
    
    try:
        while True:
            await websocket.receive_text()
            
    except WebSocketDisconnect:
        node.websocket_client = None
        node.get_logger().info("WebSocket client disconnected")
    except Exception as e:
        node.get_logger().error(f"Error in websocket connection: {str(e)}")
        node.websocket_client = None
