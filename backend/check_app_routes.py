from app.main import app

print('All routes in FastAPI app:')
for route in app.routes:
    if hasattr(route, 'methods') and hasattr(route, 'path'):
        print(f'  {list(route.methods)} {route.path}')
