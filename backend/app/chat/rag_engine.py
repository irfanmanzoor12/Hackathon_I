"""
RAG Engine - Hybrid Embeddings (Local/OpenAI) with MCP Context7
Multi-context retrieval-augmented generation
"""

from openai import AsyncOpenAI
from typing import List, Dict, Optional
import logging

from app.config import settings
import app.db_selector as db_selector
from app.db_selector import get_db_module

logger = logging.getLogger(__name__)

# OpenAI client
openai_client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)

# Local embedding model (lazy load)
_local_embedding_model = None

def get_local_embedding_model():
    """Lazy load local embedding model"""
    global _local_embedding_model
    if _local_embedding_model is None:
        from sentence_transformers import SentenceTransformer
        logger.info(f"Loading local embedding model: {settings.LOCAL_EMBEDDING_MODEL}")
        _local_embedding_model = SentenceTransformer(settings.LOCAL_EMBEDDING_MODEL)
    return _local_embedding_model

class MCPContext7Manager:
    """
    MCP Context7 - Multi-Context Protocol for 7-turn conversation memory
    Maintains context across multiple turns and contexts (selected text, history, user profile)
    """

    def __init__(self):
        self.max_history = 7

    async def build_context(
        self,
        session_id: str,
        current_query: str,
        selected_text: Optional[str] = None,
        user_profile: Optional[Dict] = None
    ) -> List[Dict[str, str]]:
        """Build multi-context conversation history"""

        # Get conversation history (last 7 turns)
        db = get_db_module()
        history = await db.get_conversation_history(session_id, limit=self.max_history)

        # Build messages array
        messages = []

        # System message with user profile context
        system_content = self._build_system_prompt(user_profile, selected_text)
        messages.append({"role": "system", "content": system_content})

        # Add conversation history
        for turn in history:
            messages.append({
                "role": turn["role"],
                "content": turn["content"]
            })

        # Add current query
        if selected_text:
            query_content = f"Selected Text: {selected_text}\n\nQuestion: {current_query}"
        else:
            query_content = current_query

        messages.append({"role": "user", "content": query_content})

        return messages

    def _build_system_prompt(self, user_profile: Optional[Dict], selected_text: Optional[str]) -> str:
        """Build system prompt with context"""

        base_prompt = """You are an expert AI tutor for Physical AI and Humanoid Robotics (Chapter 1).

Your role:
- Answer questions about ROS 2, Gazebo, Unity, NVIDIA Isaac, and embodied intelligence
- Explain robotics concepts clearly and concisely
- Provide code examples when relevant
- Reference specific weeks/modules when appropriate
- Be encouraging and supportive"""

        # Add user profile context
        if user_profile:
            software_bg = user_profile.get("software_background", "unknown")
            hardware_bg = user_profile.get("hardware_background", "unknown")

            base_prompt += f"\n\nStudent Profile:\n- Software Background: {software_bg}\n- Hardware Background: {hardware_bg}"
            base_prompt += "\n- Adjust explanations based on their background level"

        # Add selected text context
        if selected_text:
            base_prompt += "\n\nContext Mode: The student has selected specific text from the chapter. Focus your answer on explaining or expanding the selected content."

        return base_prompt


class RAGEngine:
    """RAG Engine for Chapter 1 content"""

    def __init__(self):
        self.context_manager = MCPContext7Manager()

    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for text - Hybrid mode (local or OpenAI)"""
        try:
            if settings.EMBEDDING_PROVIDER == "local":
                # Use local sentence-transformers (FREE & FAST)
                model = get_local_embedding_model()
                embedding = model.encode(text, convert_to_tensor=False)
                logger.info(f"Generated LOCAL embedding (dim: {len(embedding)})")
                return embedding.tolist()
            else:
                # Use OpenAI embeddings (CLOUD & COSTS MONEY)
                response = await openai_client.embeddings.create(
                    model=settings.OPENAI_EMBEDDING_MODEL,
                    input=text
                )
                logger.info(f"Generated OPENAI embedding (dim: {len(response.data[0].embedding)})")
                return response.data[0].embedding
        except Exception as e:
            logger.error(f"Embedding generation failed: {str(e)}")
            raise

    async def retrieve_relevant_content(
        self,
        query: str,
        selected_text: Optional[str] = None,
        limit: int = 5
    ) -> List[Dict]:
        """Retrieve relevant content from vector store"""

        # If selected text is provided, use it directly instead of retrieval
        if selected_text:
            return [{
                "content": selected_text,
                "score": 1.0,
                "section": "Selected Text",
                "metadata": {"source": "user_selection"}
            }]

        # Check if Qdrant is available
        if db_selector.use_local_qdrant:
            # Fallback: return empty results (no vector search available)
            logger.info("Qdrant unavailable, using fallback mode (no retrieval)")
            return []

        try:
            # Generate query embedding
            query_embedding = await self.generate_embedding(query)

            # Search in Qdrant
            from app.database.qdrant import search_similar
            results = search_similar(query_embedding, limit=limit)
            return results
        except Exception as e:
            logger.warning(f"Qdrant search failed: {str(e)}, using fallback")
            return []

    async def generate_response(
        self,
        user_id: int,
        session_id: str,
        query: str,
        selected_text: Optional[str] = None,
        user_profile: Optional[Dict] = None
    ) -> Dict:
        """Generate RAG response"""

        try:
            # Step 1: Retrieve relevant content
            retrieved_docs = await self.retrieve_relevant_content(query, selected_text, limit=3)

            # Step 2: Build context with MCP Context7
            messages = await self.context_manager.build_context(
                session_id, query, selected_text, user_profile
            )

            # Step 3: Add retrieved content as context
            if retrieved_docs:
                context_content = "\n\n---\n\n".join([
                    f"[{doc['section']}] (Relevance: {doc['score']:.2f})\n{doc['content']}"
                    for doc in retrieved_docs
                ])

                # Insert before user query
                messages.insert(-1, {
                    "role": "system",
                    "content": f"Retrieved Context:\n\n{context_content}"
                })

            # Step 4: Generate response with OpenAI
            response = await openai_client.chat.completions.create(
                model=settings.OPENAI_MODEL,
                messages=messages,
                temperature=0.7,
                max_tokens=1000
            )

            assistant_message = response.choices[0].message.content

            # Step 5: Save conversation to database
            db = get_db_module()
            await db.save_conversation(user_id, session_id, "user", query)
            await db.save_conversation(user_id, session_id, "assistant", assistant_message)

            return {
                "response": assistant_message,
                "sources": retrieved_docs,
                "tokens_used": response.usage.total_tokens
            }

        except Exception as e:
            logger.error(f"RAG generation failed: {str(e)}")
            raise


# Global RAG engine instance
rag_engine = RAGEngine()
