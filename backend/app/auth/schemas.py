"""
Authentication Schemas - Pydantic Models
"""

from pydantic import BaseModel, EmailStr
from typing import Optional
from datetime import datetime


class UserCreate(BaseModel):
    """User creation schema"""
    email: EmailStr
    name: str
    picture: Optional[str] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None


class UserRegister(BaseModel):
    """User registration with email/password"""
    email: EmailStr
    password: str
    name: str
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None


class UserLogin(BaseModel):
    """User login with email/password"""
    email: EmailStr
    password: str


class UserUpdate(BaseModel):
    """User update schema"""
    name: Optional[str] = None
    picture: Optional[str] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None


class UserResponse(BaseModel):
    """User response schema"""
    id: int
    email: str
    name: str
    picture: Optional[str] = None
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None
    created_at: datetime
    last_login: Optional[datetime] = None


class TokenResponse(BaseModel):
    """Token response schema"""
    access_token: str
    token_type: str = "bearer"
    user: UserResponse
