"""
Reusable Subagents - Personalization and Code Explanation
"""

from openai import AsyncOpenAI
from typing import Dict, Optional
import logging

from app.config import settings

logger = logging.getLogger(__name__)
openai_client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)


class ITBackgroundPersonalizer:
    """
    Subagent: IT Background Personalizer
    Personalizes responses based on user's software/hardware background
    """

    async def personalize(self, content: str, user_profile: Dict) -> str:
        """Personalize content based on user background"""

        software_bg = user_profile.get("software_background", "beginner")
        hardware_bg = user_profile.get("hardware_background", "beginner")

        prompt = f"""You are a personalization expert. Adapt the following content based on the user's background:

User Profile:
- Software Background: {software_bg}
- Hardware Background: {hardware_bg}

Original Content:
{content}

Task: Rewrite this content to match the user's experience level.
- If beginner: Add more foundational explanations and analogies
- If intermediate: Add practical examples and best practices
- If advanced: Add deeper technical details and optimization tips

Keep the same core information but adjust the depth and style."""

        try:
            response = await openai_client.chat.completions.create(
                model=settings.OPENAI_MODEL,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.7,
                max_tokens=800
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Personalization failed: {str(e)}")
            return content  # Return original if personalization fails


class CodeExplainer:
    """
    Subagent: Code Explainer
    Explains code snippets and robotics logic
    """

    async def explain(self, code: str, context: Optional[str] = None) -> str:
        """Explain code snippet"""

        prompt = f"""You are a robotics code expert. Explain this code clearly and concisely.

{f"Context: {context}" if context else ""}

Code:
```
{code}
```

Provide:
1. What this code does (1-2 sentences)
2. Key components explained line-by-line or block-by-block
3. Why this pattern is used in robotics/Physical AI
4. Common pitfalls or best practices

Keep explanations clear and practical."""

        try:
            response = await openai_client.chat.completions.create(
                model=settings.OPENAI_MODEL,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.6,
                max_tokens=1000
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Code explanation failed: {str(e)}")
            return "Unable to explain code at this time."


class TranslationAgent:
    """
    Subagent: Translation Agent
    Translates content to Urdu or other languages
    """

    async def translate(self, content: str, target_language: str = "Urdu") -> str:
        """Translate content to target language"""

        prompt = f"""Translate the following technical content to {target_language}.

Important:
- Maintain technical terms in English (e.g., ROS 2, NVIDIA Isaac, Gazebo)
- Keep code snippets unchanged
- Preserve markdown formatting
- Ensure technical accuracy

Content:
{content}"""

        try:
            response = await openai_client.chat.completions.create(
                model=settings.OPENAI_MODEL,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.3,
                max_tokens=1500
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Translation failed: {str(e)}")
            return content  # Return original if translation fails


# Global subagent instances
personalizer = ITBackgroundPersonalizer()
code_explainer = CodeExplainer()
translator = TranslationAgent()
