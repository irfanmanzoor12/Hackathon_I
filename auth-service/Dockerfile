# Auth Service Dockerfile
FROM node:20-alpine

WORKDIR /app

# Install dependencies
COPY package.json ./
RUN npm install --production

# Copy source code
COPY src ./src

# Expose port
EXPOSE 3001

# Run the service
CMD ["npm", "start"]
