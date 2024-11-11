# ROS2 Communication Server

A FastAPI-based server that bridges ROS2 communication with HTTP/WebSocket protocols, allowing external applications to interact with ROS2 topics and services through a RESTful API and WebSocket connections.

## Package Structure
```
communication_server/
├── communication_server/          # Main package directory
│   ├── __init__.py
│   ├── communication_server.py    # Main server implementation
│   └── routes/                    # Route definitions
│       ├── __init__.py
│       ├── admin_api.py          # Administrative REST endpoints
│       └── websocket.py          # WebSocket endpoint definitions
├── test/                         # Test directory
│   ├── test_comms.py            # Communication tests
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── resource/                     # Package resources
├── package.xml                   # Package manifest
├── setup.cfg                     # Package configuration
└── setup.py                      # Package setup script
```

## Creating New Endpoints

### Adding REST API Endpoints
Create endpoints by creating a new file in `routes/` and adding the route definitions:

```python
from fastapi import APIRouter, HTTPException
from typing import Dict, Any

router = APIRouter()

def get_node_manager():
    from communication_server.communication_server import node_manager
    return node_manager

@router.post("/api/topics")
async def create_topic(data: Dict[str, Any]):
    """Example endpoint for creating a new topic"""
    node_manager = get_node_manager()
    try:
        # Your endpoint logic here
        return {"status": "success"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

### Adding WebSocket Endpoints
Create endpoints by adding a new WebSocket route in `routes/websocket.py`:
```python
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import Dict, Any

router = APIRouter()

def get_node_manager():
    from communication_server.communication_server import node_manager
    return node_manager

@router.websocket("/ws/custom")
async def custom_websocket_endpoint(websocket: WebSocket):
    """Example custom WebSocket endpoint"""
    node_manager = get_node_manager()
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_json()
            # Handle WebSocket data
    except WebSocketDisconnect:
        # Handle disconnect
```

### Registering New Endpoints
Add the new route to the main FastAPI app in `routes/__init__.py`:
```python
from fastapi import APIRouter
from .example_api import router as example_router

def get_routers():
    router = APIRouter()
    # Add new routes here
    router.include_router(example_router, prefix="/example")
    return router
```

## Core Functionality Offered

### CommunicationNode
The main ROS2 node that handles all ROS2 communications. Key features:

1. **Publishers Management**:
```python
# Add a new publisher
node.add_publisher(topic_name, String)

# Publish a message
node.publish_message(topic_name, message)
```

2. **Subscribers Management**:
```python
# Add a new subscription
node.subscribers.append(
    node.create_subscription(
        String,
        topic_name,
        partial(node.message_callback, topic=topic_name),
        10
    )
)
```

3. **Service Client Management**:
```python
# Add a service client
node.add_service_client(service_name, service_type)

# Call a service asynchronously
response = await node.call_service(service_name, request)
```

## Best Practices

1. **Thread Safety**:
   - Use the lock when modifying shared resources:
   ```python
   with node.lock:
       # Modify shared resources
   ```

2. **Async/Sync Boundary**:
   - Use the message queue for communication between sync ROS2 callbacks and async code
   - Don't call async functions directly from synchronous callbacks

3. **Error Handling**:
   ```python
   try:
       # Your endpoint logic
   except Exception as e:
       raise HTTPException(status_code=500, detail=str(e))
   ```

4. **Resource Management**:
   - Clean up resources in WebSocket disconnect handlers
   - Use background task management for async operations

For more information on FastAPI routing and best practices, refer to the [FastAPI documentation](https://fastapi.tiangolo.com/tutorial/bigger-applications/).