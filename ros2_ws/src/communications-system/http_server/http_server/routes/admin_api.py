from fastapi import APIRouter
from functools import partial
from std_msgs.msg import String

router = APIRouter()

def get_node_manager():
    from http_server.http_server import node_manager
    return node_manager

@router.get("/test")
async def read_root():
    return {"message": "Hello World"}

@router.post("/add_subscription")
async def add_subscription(subscription: dict):
    node_manager = get_node_manager()
    topic = subscription["topic"]
    if topic not in node_manager.node.subscribed_topics:
        node_manager.node.subscribed_topics.append(topic)
        node_manager.node.subscribers.append(
            node_manager.node.create_subscription(
                String,
                topic,
                partial(node_manager.node.message_callback, topic=topic),
                10,
            )
        )
    return {"status": "success"}