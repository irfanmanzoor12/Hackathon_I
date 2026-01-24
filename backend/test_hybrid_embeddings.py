"""
Test Hybrid Embeddings - Quick verification script
Run: python test_hybrid_embeddings.py
"""

import asyncio
import sys
import os

# Add backend to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from app.config import settings
from app.chat.rag_engine import rag_engine


async def test_embeddings():
    """Test embedding generation"""

    print("\n" + "="*60)
    print("HYBRID EMBEDDINGS TEST")
    print("="*60)

    print(f"\n📊 Configuration:")
    print(f"   Embedding Provider: {settings.EMBEDDING_PROVIDER}")
    print(f"   Vector Dimension: {settings.VECTOR_DIMENSION}")

    if settings.EMBEDDING_PROVIDER == "local":
        print(f"   Local Model: {settings.LOCAL_EMBEDDING_MODEL}")
    else:
        print(f"   OpenAI Model: {settings.OPENAI_EMBEDDING_MODEL}")

    # Test query
    test_query = "What is ROS 2 and how does it work?"

    print(f"\n🧪 Testing embedding generation...")
    print(f"   Query: '{test_query}'")

    try:
        # Generate embedding
        embedding = await rag_engine.generate_embedding(test_query)

        print(f"\n✅ SUCCESS!")
        print(f"   Embedding dimension: {len(embedding)}")
        print(f"   First 5 values: {embedding[:5]}")

        if settings.EMBEDDING_PROVIDER == "local":
            print(f"\n💰 Cost: $0 (FREE!)")
            print(f"   You're saving money with local embeddings!")
        else:
            print(f"\n💰 Cost: ~$0.0001 per request")
            print(f"   Using cloud OpenAI embeddings")

        print("\n" + "="*60)
        print("✅ Hybrid embeddings are working correctly!")
        print("="*60 + "\n")

    except Exception as e:
        print(f"\n❌ ERROR: {str(e)}")
        print(f"\nTroubleshooting:")
        print(f"1. Install dependencies: pip install sentence-transformers torch")
        print(f"2. Check .env file has: EMBEDDING_PROVIDER=local")
        print(f"3. If using OpenAI, ensure OPENAI_API_KEY is set")
        print("\n" + "="*60 + "\n")
        return False

    return True


if __name__ == "__main__":
    # Run test
    success = asyncio.run(test_embeddings())

    if success:
        print("🚀 Ready to start the backend:")
        print("   cd backend")
        print("   uvicorn app.main:app --reload")
        sys.exit(0)
    else:
        sys.exit(1)
