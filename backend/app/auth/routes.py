"""
Authentication Routes - Google OAuth 2.0
"""

from fastapi import APIRouter, HTTPException, status, Request
from fastapi.responses import RedirectResponse
from authlib.integrations.starlette_client import OAuth
from datetime import datetime, timedelta
import jwt
import httpx
import logging

from app.config import settings
from app.auth.schemas import UserCreate, UserRegister, UserLogin, UserResponse, TokenResponse
from app.db_selector import get_db_module, use_local_db

logger = logging.getLogger(__name__)
router = APIRouter()

# OAuth Configuration
oauth = OAuth()
oauth.register(
    name='google',
    client_id=settings.GOOGLE_CLIENT_ID,
    client_secret=settings.GOOGLE_CLIENT_SECRET,
    server_metadata_url='https://accounts.google.com/.well-known/openid-configuration',
    client_kwargs={'scope': 'openid email profile'}
)


def create_access_token(data: dict) -> str:
    """Create JWT access token"""
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, settings.SECRET_KEY, algorithm=settings.ALGORITHM)
    return encoded_jwt


def verify_token(token: str) -> dict:
    """Verify JWT token"""
    try:
        payload = jwt.decode(token, settings.SECRET_KEY, algorithms=[settings.ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Token has expired")
    except jwt.JWTError:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid token")


@router.get("/google/login")
async def google_login(request: Request):
    """Initiate Google OAuth login"""
    redirect_uri = settings.GOOGLE_REDIRECT_URI
    return await oauth.google.authorize_redirect(request, redirect_uri)


@router.get("/google/callback")
async def google_callback(request: Request):
    """Handle Google OAuth callback"""
    try:
        db = get_db_module()

        # Get token from Google
        token = await oauth.google.authorize_access_token(request)

        # Get user info from Google
        user_info = token.get('userinfo')
        if not user_info:
            raise HTTPException(status_code=400, detail="Failed to get user info")

        email = user_info.get('email')
        name = user_info.get('name')
        picture = user_info.get('picture')

        if not email:
            raise HTTPException(status_code=400, detail="Email not provided by Google")

        # Check if user exists
        user = await db.get_user_by_email(email)

        if not user:
            # Create new user
            user = await db.create_user(
                email=email,
                name=name,
                picture=picture
            )
            logger.info(f"New user created: {email}")
        else:
            # Update existing user
            await db.update_user(user['id'], {
                'name': name,
                'picture': picture,
                'last_login': datetime.utcnow()
            })
            logger.info(f"User logged in: {email}")

        # Create JWT token
        access_token = create_access_token({
            "sub": str(user['id']),
            "email": email,
            "name": name
        })

        # Redirect to frontend with token
        frontend_url = f"{settings.FRONTEND_URL}/auth/callback?token={access_token}"
        return RedirectResponse(url=frontend_url)

    except Exception as e:
        logger.error(f"OAuth callback error: {str(e)}")
        error_url = f"{settings.FRONTEND_URL}/auth/error?message={str(e)}"
        return RedirectResponse(url=error_url)


@router.get("/verify")
async def verify_user_token(token: str):
    """Verify JWT token and return user info"""
    payload = verify_token(token)
    return {
        "valid": True,
        "user_id": payload.get("sub"),
        "email": payload.get("email"),
        "name": payload.get("name")
    }


@router.post("/logout")
async def logout():
    """Logout user (client-side token removal)"""
    return {"message": "Logged out successfully"}


@router.post("/register")
async def register_user(user_data: UserRegister):
    """Register new user with email/password"""
    try:
        db = get_db_module()

        # Check if user already exists
        existing_user = await db.get_user_by_email(user_data.email)
        if existing_user:
            raise HTTPException(status_code=400, detail="Email already registered")

        # Create new user
        user = await db.create_user(
            email=user_data.email,
            name=user_data.name,
            password=user_data.password,
            software_background=user_data.software_background,
            hardware_background=user_data.hardware_background
        )

        # Create JWT token
        access_token = create_access_token({
            "sub": str(user['id']),
            "email": user['email'],
            "name": user['name']
        })

        logger.info(f"New user registered: {user_data.email}")

        return {
            "access_token": access_token,
            "token_type": "bearer",
            "user": {
                "id": user['id'],
                "email": user['email'],
                "name": user['name']
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        import traceback
        logger.error(f"Registration error: {str(e)}")
        logger.error(f"Traceback: {traceback.format_exc()}")
        raise HTTPException(status_code=500, detail=f"Registration failed: {str(e)}")


@router.post("/login")
async def login_user(credentials: UserLogin):
    """Login user with email/password"""
    try:
        db = get_db_module()

        # Get user by email
        user = await db.get_user_by_email(credentials.email)
        if not user:
            raise HTTPException(status_code=401, detail="Invalid email or password")

        # Verify password
        if not user.get('password_hash'):
            raise HTTPException(status_code=401, detail="Please use Google Sign-In for this account")

        if use_local_db:
            from app.database.sqlite_local import verify_password
        else:
            # Add password verification to postgres module if needed
            raise HTTPException(status_code=501, detail="Password login not supported with cloud database")

        if not verify_password(credentials.password, user['password_hash']):
            raise HTTPException(status_code=401, detail="Invalid email or password")

        # Update last login
        await db.update_user(user['id'], {'last_login': datetime.utcnow()})

        # Create JWT token
        access_token = create_access_token({
            "sub": str(user['id']),
            "email": user['email'],
            "name": user['name']
        })

        logger.info(f"User logged in: {credentials.email}")

        return {
            "access_token": access_token,
            "token_type": "bearer",
            "user": {
                "id": user['id'],
                "email": user['email'],
                "name": user['name']
            }
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Login error: {str(e)}")
        raise HTTPException(status_code=500, detail="Login failed")
