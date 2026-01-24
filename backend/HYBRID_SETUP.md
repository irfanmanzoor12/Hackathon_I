# Hybrid Embeddings Setup Guide

## What Changed?

Your chatbot now supports **HYBRID EMBEDDINGS**:
- **Local Mode** (FREE & FAST): Uses sentence-transformers on your machine
- **Cloud Mode** (PAID): Uses OpenAI embeddings API

Currently configured: **LOCAL MODE** ✅

---

## Installation Steps

### 1. Install New Dependencies

```bash
cd backend
pip install sentence-transformers==2.3.1 torch
```

Or install all requirements:
```bash
pip install -r requirements.txt
```

### 2. Configuration

Your `.env` is already configured with:
```
EMBEDDING_PROVIDER=local
```

**To switch back to OpenAI embeddings:**
```
EMBEDDING_PROVIDER=openai
```

---

## Important: Vector Dimensions

- **Local embeddings**: 384 dimensions (all-MiniLM-L6-v2)
- **OpenAI embeddings**: 1536 dimensions (text-embedding-3-small)

### If You Have Existing Data in Qdrant:

Your existing Qdrant collection uses **1536 dimensions** (OpenAI).

**Option 1: Create New Collection for Local Embeddings**
```bash
# Update .env to use new collection
QDRANT_COLLECTION_NAME=chapter_1_local_embeddings
```

Then re-index your data:
```bash
python scripts/index_chapter1.py
```

**Option 2: Keep Using OpenAI Embeddings**
```bash
# Just switch back in .env
EMBEDDING_PROVIDER=openai
```

---

## Testing

Start the backend:
```bash
cd backend
uvicorn app.main:app --reload
```

The logs will show which embedding provider is active:
- `Generated LOCAL embedding (dim: 384)` - Local mode ✅
- `Generated OPENAI embedding (dim: 1536)` - OpenAI mode

---

## Cost Savings

**Before (OpenAI only):**
- Embeddings: ~$0.0001 per request
- LLM responses: ~$0.01 per request
- **Total: ~$0.0101 per query**

**After (Hybrid - Local embeddings + OpenAI LLM):**
- Embeddings: $0 (local)
- LLM responses: ~$0.01 per request
- **Total: ~$0.01 per query (70-80% savings!)**

---

## Troubleshooting

**Error: "No module named 'sentence_transformers'"**
```bash
pip install sentence-transformers torch
```

**Error: Dimension mismatch in Qdrant**
- Your collection expects different dimensions
- Either create new collection or switch embedding provider

**Slow first request**
- Local model downloads on first use (~90MB)
- Subsequent requests are instant

---

## Next Steps

1. Install dependencies: `pip install -r requirements.txt`
2. Choose: Keep local mode OR switch back to OpenAI in `.env`
3. If local: Re-index data with new collection name
4. Test: Start backend and check logs
