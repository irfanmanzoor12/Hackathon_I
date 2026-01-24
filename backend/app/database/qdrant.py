"""
Qdrant Vector Store - Chapter 1 Content Embeddings
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from typing import List, Dict, Any, Optional
import logging
import uuid

from app.config import settings

logger = logging.getLogger(__name__)

# Qdrant client
qdrant_client: Optional[QdrantClient] = None


async def init_qdrant():
    """Initialize Qdrant client and collection"""
    global qdrant_client

    try:
        qdrant_client = QdrantClient(
            url=settings.QDRANT_URL,
            api_key=settings.QDRANT_API_KEY,
            timeout=30
        )

        # Check if collection exists
        collections = qdrant_client.get_collections().collections
        collection_names = [col.name for col in collections]

        if settings.QDRANT_COLLECTION_NAME not in collection_names:
            # Create collection
            qdrant_client.create_collection(
                collection_name=settings.QDRANT_COLLECTION_NAME,
                vectors_config=VectorParams(
                    size=settings.VECTOR_DIMENSION,
                    distance=Distance.COSINE
                )
            )
            logger.info(f"✅ Created Qdrant collection: {settings.QDRANT_COLLECTION_NAME}")
        else:
            logger.info(f"✅ Qdrant collection exists: {settings.QDRANT_COLLECTION_NAME}")

        logger.info("✅ Qdrant client initialized")

    except Exception as e:
        logger.error(f"❌ Failed to initialize Qdrant: {str(e)}")
        raise


async def close_qdrant():
    """Close Qdrant client"""
    global qdrant_client
    if qdrant_client:
        qdrant_client.close()
        logger.info("✅ Qdrant client closed")


def upsert_documents(documents: List[Dict[str, Any]], embeddings: List[List[float]]):
    """Insert or update documents in Qdrant"""
    points = []

    for idx, (doc, embedding) in enumerate(zip(documents, embeddings)):
        point = PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload=doc
        )
        points.append(point)

    qdrant_client.upsert(
        collection_name=settings.QDRANT_COLLECTION_NAME,
        points=points
    )

    logger.info(f"✅ Upserted {len(points)} documents to Qdrant")


def search_similar(query_embedding: List[float], limit: int = 5, filter_dict: Optional[Dict] = None) -> List[Dict]:
    """Search for similar documents"""
    search_params = {
        "collection_name": settings.QDRANT_COLLECTION_NAME,
        "query_vector": query_embedding,
        "limit": limit
    }

    if filter_dict:
        search_params["query_filter"] = Filter(
            must=[
                FieldCondition(
                    key=key,
                    match=MatchValue(value=value)
                ) for key, value in filter_dict.items()
            ]
        )

    results = qdrant_client.search(**search_params)

    # Convert to dict format
    documents = []
    for result in results:
        doc = {
            "id": result.id,
            "score": result.score,
            "content": result.payload.get("content", ""),
            "metadata": result.payload.get("metadata", {}),
            "section": result.payload.get("section", ""),
            "week": result.payload.get("week", "")
        }
        documents.append(doc)

    return documents


def get_collection_info() -> Dict:
    """Get collection information"""
    try:
        info = qdrant_client.get_collection(settings.QDRANT_COLLECTION_NAME)
        return {
            "name": info.name,
            "vectors_count": info.vectors_count,
            "points_count": info.points_count,
            "status": info.status
        }
    except Exception as e:
        logger.error(f"Failed to get collection info: {str(e)}")
        return {}
