from fastapi import APIRouter
from .admin_api import router as admin_router
from .websocket import router as websocket_router
from .cameras import router as cameras_router

def get_routers():
    router = APIRouter()
    router.include_router(admin_router, prefix="/admin", tags=["api"])
    router.include_router(websocket_router, tags=["websocket"])
    router.include_router(cameras_router, tags=["api"])
    return router