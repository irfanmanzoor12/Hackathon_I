"""
Chat Routes - RAG Chatbot Endpoints
"""

from fastapi import APIRouter, HTTPException, Depends, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel
from typing import Optional
import uuid
import logging

from app.auth.routes import verify_token
from app.db_selector import get_db_module
from app.chat.rag_engine import rag_engine
from app.chat.subagents import personalizer, code_explainer, translator

logger = logging.getLogger(__name__)
router = APIRouter()
security = HTTPBearer()


class ChatRequest(BaseModel):
    """Chat request schema"""
    message: str
    session_id: Optional[str] = None
    selected_text: Optional[str] = None
    action: Optional[str] = None  # "personalize", "translate", "explain_code"


class ChatResponse(BaseModel):
    """Chat response schema"""
    response: str
    session_id: str
    sources: list
    tokens_used: int


class ProfileUpdateRequest(BaseModel):
    """User profile update schema"""
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None


async def get_current_user(credentials: HTTPAuthorizationCredentials = Depends(security)):
    """Get current user from JWT token"""
    db = get_db_module()
    token = credentials.credentials
    payload = verify_token(token)
    user_id = int(payload.get("sub"))

    user = await db.get_user_by_id(user_id)
    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    return user


@router.post("/message", response_model=ChatResponse)
async def chat_message(request: ChatRequest, user: dict = Depends(get_current_user)):
    """
    Main chat endpoint - RAG-powered conversation
    """
    try:
        # Generate or use existing session ID
        session_id = request.session_id or str(uuid.uuid4())

        # Get user profile for personalization
        user_profile = {
            "software_background": user.get("software_background"),
            "hardware_background": user.get("hardware_background")
        }

        # Generate RAG response
        result = await rag_engine.generate_response(
            user_id=user["id"],
            session_id=session_id,
            query=request.message,
            selected_text=request.selected_text,
            user_profile=user_profile
        )

        # Apply action if specified
        if request.action == "personalize" and user_profile.get("software_background"):
            result["response"] = await personalizer.personalize(
                result["response"],
                user_profile
            )
        elif request.action == "translate":
            result["response"] = await translator.translate(result["response"], "Urdu")
        elif request.action == "explain_code" and request.selected_text:
            result["response"] = await code_explainer.explain(
                request.selected_text,
                context=request.message
            )

        return ChatResponse(
            response=result["response"],
            session_id=session_id,
            sources=result["sources"],
            tokens_used=result["tokens_used"]
        )

    except Exception as e:
        logger.error(f"Chat error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/personalize")
async def personalize_content(request: ChatRequest, user: dict = Depends(get_current_user)):
    """
    Personalize content based on user IT background
    """
    try:
        user_profile = {
            "software_background": user.get("software_background", "beginner"),
            "hardware_background": user.get("hardware_background", "beginner")
        }

        personalized = await personalizer.personalize(request.message, user_profile)

        return {"response": personalized, "personalized_content": personalized}

    except Exception as e:
        logger.error(f"Personalization error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/translate")
async def translate_content(request: ChatRequest, user: dict = Depends(get_current_user)):
    """
    Translate content to Urdu
    """
    try:
        translated = await translator.translate(request.message, "Urdu")
        return {"response": translated, "translated_content": translated}

    except Exception as e:
        logger.error(f"Translation error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/explain-code")
async def explain_code(request: ChatRequest, user: dict = Depends(get_current_user)):
    """
    Explain code snippet
    """
    try:
        if not request.selected_text:
            raise HTTPException(status_code=400, detail="No code provided")

        explanation = await code_explainer.explain(
            request.selected_text,
            context=request.message
        )

        return {"explanation": explanation}

    except Exception as e:
        logger.error(f"Code explanation error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.put("/profile")
async def update_profile(
    profile_data: ProfileUpdateRequest,
    user: dict = Depends(get_current_user)
):
    """
    Update user profile (software/hardware background)
    """
    try:
        db = get_db_module()
        updated_user = await db.update_user(user["id"], profile_data.dict(exclude_none=True))
        return {"message": "Profile updated", "user": updated_user}

    except Exception as e:
        logger.error(f"Profile update error: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/profile")
async def get_profile(user: dict = Depends(get_current_user)):
    """
    Get user profile
    """
    return {
        "id": user["id"],
        "email": user["email"],
        "name": user["name"],
        "software_background": user.get("software_background"),
        "hardware_background": user.get("hardware_background")
    }
