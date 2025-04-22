from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from std_msgs.msg import String
import json

router = APIRouter()

def get_node_manager():
    from communication_server.communication_server import node_manager
    return node_manager

@router.websocket("/ws_publish")
async def websocket_publish(websocket: WebSocket):
    nm = get_node_manager()
    await websocket.accept()
    nm.node.publish_ws = websocket
    try:
        while True:
            data = await websocket.receive_text()
            msg = json.loads(data)
            topic = msg.get("topic")
            message = msg.get("message")
            if topic and message:
                if not any(pub.topic_name == topic for pub in nm.node._publishers):
                    nm.node.add_publisher(topic, String)
                nm.node.publish_message(topic, message)
    except WebSocketDisconnect:
        nm.node.publish_ws = None

@router.websocket("/ws_sub")
async def websocket_subscribe(websocket: WebSocket):
    nm = get_node_manager()
    await websocket.accept()
    nm.node.subscribe_ws = websocket
    try:
        while True:
            data = await websocket.receive_text()
            msg = json.loads(data)
            topic = msg.get("topic")
            
            if topic and topic not in nm.node.subscribed_topics:
                nm.node.subscribed_topics.append(topic)
                nm.node.get_logger().info(f"Added subscription to topic: {topic}")
                nm.node.init_subscribers()
    except WebSocketDisconnect:
        nm.node.subscribe_ws = None