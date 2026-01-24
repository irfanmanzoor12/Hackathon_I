from app.auth.routes import router

print('Routes in auth router:')
for route in router.routes:
    print(f'  {list(route.methods)} {route.path}')
