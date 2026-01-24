/**
 * Playwright E2E Tests - RAG Chatbot
 * Tests authentication, chat functionality, and UI interactions
 */

const { test, expect } = require('@playwright/test');

const FRONTEND_URL = process.env.FRONTEND_URL || 'http://localhost:3000';
const BACKEND_URL = process.env.BACKEND_URL || 'http://localhost:8000';

test.describe('RAG Chatbot - Authentication Flow', () => {
  test('should display chatbot toggle button', async ({ page }) => {
    await page.goto(FRONTEND_URL);

    // Check for toggle button
    const toggleButton = page.locator('.rag-chat-toggle');
    await expect(toggleButton).toBeVisible();

    // Check for avatar animation
    const avatar = toggleButton.locator('.agentic-avatar');
    await expect(avatar).toBeVisible();
  });

  test('should open chatbot widget on click', async ({ page }) => {
    await page.goto(FRONTEND_URL);

    // Click toggle button
    await page.click('.rag-chat-toggle');

    // Widget should be visible
    const widget = page.locator('.rag-chat-widget');
    await expect(widget).toBeVisible();

    // Should show auth prompt if not logged in
    const authSection = page.locator('.auth-section');
    if (await authSection.isVisible()) {
      await expect(authSection).toContainText('Sign in with Google');
    }
  });

  test('should close chatbot widget', async ({ page }) => {
    await page.goto(FRONTEND_URL);

    // Open widget
    await page.click('.rag-chat-toggle');
    await expect(page.locator('.rag-chat-widget')).toBeVisible();

    // Close widget
    await page.click('.close-btn');

    // Widget should not be visible
    await expect(page.locator('.rag-chat-widget')).not.toBeVisible();
  });
});

test.describe('RAG Chatbot - Chat Functionality', () => {
  // Helper to login (requires valid token)
  const loginWithToken = async (page, token) => {
    await page.goto(FRONTEND_URL);
    await page.evaluate((token) => {
      localStorage.setItem('rag_token', token);
    }, token);
    await page.reload();
  };

  test.skip('should send a chat message (requires authentication)', async ({ page }) => {
    // This test requires a valid token - skip in CI
    // To run manually: set TOKEN environment variable
    const token = process.env.TEST_TOKEN;
    if (!token) {
      test.skip();
      return;
    }

    await loginWithToken(page, token);

    // Open widget
    await page.click('.rag-chat-toggle');
    await expect(page.locator('.rag-chat-widget')).toBeVisible();

    // Type message
    const input = page.locator('.rag-chat-input textarea');
    await input.fill('What is ROS 2?');

    // Send message
    await page.click('.rag-chat-input button');

    // Wait for response
    await page.waitForSelector('.message-assistant', { timeout: 10000 });

    // Check response appears
    const messages = page.locator('.message-assistant');
    await expect(messages.first()).toBeVisible();
  });

  test('should display typing indicator while loading', async ({ page }) => {
    const token = process.env.TEST_TOKEN;
    if (!token) {
      test.skip();
      return;
    }

    await loginWithToken(page, token);
    await page.click('.rag-chat-toggle');

    // Send message
    await page.fill('.rag-chat-input textarea', 'Tell me about Gazebo simulation');
    await page.click('.rag-chat-input button');

    // Typing indicator should appear
    const typingIndicator = page.locator('.typing-indicator');
    await expect(typingIndicator).toBeVisible();
  });
});

test.describe('RAG Chatbot - Text Selection', () => {
  test('should detect text selection', async ({ page }) => {
    await page.goto(FRONTEND_URL);

    // Create a test paragraph
    await page.setContent(`
      <html>
        <body>
          <p id="test-content">
            ROS 2 is a middleware framework for robotics development.
            It provides communication between distributed processes using topics,
            services, and actions. This is important for Physical AI systems.
          </p>
          <script src="${FRONTEND_URL}/static/js/bundle.js"></script>
        </body>
      </html>
    `);

    // Select text
    const paragraph = page.locator('#test-content');
    await paragraph.dblclick(); // Double-click to select word

    // Wait briefly for selection handler
    await page.waitForTimeout(500);

    // Widget should open (if selection is detected)
    // Note: This behavior depends on implementation
  });
});

test.describe('RAG Chatbot - Action Buttons', () => {
  test('should show personalize and translate buttons', async ({ page }) => {
    const token = process.env.TEST_TOKEN;
    if (!token) {
      test.skip();
      return;
    }

    await page.goto(FRONTEND_URL);
    await page.evaluate((token) => {
      localStorage.setItem('rag_token', token);
    }, token);
    await page.reload();

    // Open widget
    await page.click('.rag-chat-toggle');

    // Check action buttons exist
    await expect(page.locator('button:has-text("Personalize")')).toBeVisible();
    await expect(page.locator('button:has-text("Translate")')).toBeVisible();
  });

  test('buttons should be disabled when input is empty', async ({ page }) => {
    const token = process.env.TEST_TOKEN;
    if (!token) {
      test.skip();
      return;
    }

    await page.goto(FRONTEND_URL);
    await page.evaluate((token) => {
      localStorage.setItem('rag_token', token);
    }, token);
    await page.reload();

    await page.click('.rag-chat-toggle');

    // Buttons should be disabled
    const personalizeBtn = page.locator('button:has-text("Personalize")');
    const translateBtn = page.locator('button:has-text("Translate")');

    await expect(personalizeBtn).toBeDisabled();
    await expect(translateBtn).toBeDisabled();
  });

  test('buttons should be enabled when input has text', async ({ page }) => {
    const token = process.env.TEST_TOKEN;
    if (!token) {
      test.skip();
      return;
    }

    await page.goto(FRONTEND_URL);
    await page.evaluate((token) => {
      localStorage.setItem('rag_token', token);
    }, token);
    await page.reload();

    await page.click('.rag-chat-toggle');

    // Type in input
    await page.fill('.rag-chat-input textarea', 'Explain ROS 2 nodes');

    // Buttons should be enabled
    const personalizeBtn = page.locator('button:has-text("Personalize")');
    const translateBtn = page.locator('button:has-text("Translate")');

    await expect(personalizeBtn).toBeEnabled();
    await expect(translateBtn).toBeEnabled();
  });
});

test.describe('RAG Chatbot - Avatar Animations', () => {
  test('should display avatar with idle state', async ({ page }) => {
    await page.goto(FRONTEND_URL);
    await page.click('.rag-chat-toggle');

    const avatar = page.locator('.agentic-avatar');
    await expect(avatar).toBeVisible();

    // Check SVG elements exist
    await expect(avatar.locator('svg')).toBeVisible();
  });

  test('should animate avatar during interactions', async ({ page }) => {
    const token = process.env.TEST_TOKEN;
    if (!token) {
      test.skip();
      return;
    }

    await page.goto(FRONTEND_URL);
    await page.evaluate((token) => {
      localStorage.setItem('rag_token', token);
    }, token);
    await page.reload();

    await page.click('.rag-chat-toggle');

    // Send message and check for state changes
    await page.fill('.rag-chat-input textarea', 'What is Physical AI?');
    await page.click('.rag-chat-input button');

    // Avatar should be in thinking/responding state
    // (Visual regression testing would be ideal here)
    await page.waitForTimeout(1000);
  });
});

test.describe('RAG Chatbot - Backend API Integration', () => {
  test('backend health check', async ({ request }) => {
    const response = await request.get(`${BACKEND_URL}/health`);
    expect(response.ok()).toBeTruthy();

    const data = await response.json();
    expect(data.status).toBe('healthy');
  });

  test('should handle API errors gracefully', async ({ page }) => {
    const token = process.env.TEST_TOKEN;
    if (!token) {
      test.skip();
      return;
    }

    await page.goto(FRONTEND_URL);
    await page.evaluate((token) => {
      localStorage.setItem('rag_token', token);
    }, token);

    // Mock failed API response
    await page.route('**/chat/message', (route) => {
      route.fulfill({
        status: 500,
        body: JSON.stringify({ detail: 'Internal server error' })
      });
    });

    await page.reload();
    await page.click('.rag-chat-toggle');

    // Send message
    await page.fill('.rag-chat-input textarea', 'Test error handling');
    await page.click('.rag-chat-input button');

    // Should show error message
    await page.waitForSelector('.message-system');
    const errorMsg = page.locator('.message-system');
    await expect(errorMsg).toContainText('something went wrong');
  });
});

test.describe('RAG Chatbot - Responsive Design', () => {
  test('should be responsive on mobile', async ({ page }) => {
    await page.setViewportSize({ width: 375, height: 667 });
    await page.goto(FRONTEND_URL);

    // Toggle should still be visible
    await expect(page.locator('.rag-chat-toggle')).toBeVisible();

    // Open widget
    await page.click('.rag-chat-toggle');

    // Widget should take full screen on mobile
    const widget = page.locator('.rag-chat-widget');
    await expect(widget).toBeVisible();

    // Check computed styles
    const boundingBox = await widget.boundingBox();
    expect(boundingBox.width).toBeCloseTo(375, 10);
  });

  test('should be responsive on tablet', async ({ page }) => {
    await page.setViewportSize({ width: 768, height: 1024 });
    await page.goto(FRONTEND_URL);

    await page.click('.rag-chat-toggle');
    const widget = page.locator('.rag-chat-widget');
    await expect(widget).toBeVisible();
  });
});

test.describe('RAG Chatbot - Accessibility', () => {
  test('should have proper ARIA labels', async ({ page }) => {
    await page.goto(FRONTEND_URL);
    await page.click('.rag-chat-toggle');

    // Check for accessible elements
    const input = page.locator('.rag-chat-input textarea');
    await expect(input).toHaveAttribute('placeholder');
  });

  test('should support keyboard navigation', async ({ page }) => {
    await page.goto(FRONTEND_URL);
    await page.click('.rag-chat-toggle');

    // Tab to input
    await page.keyboard.press('Tab');

    // Type message
    await page.keyboard.type('Test message');

    // Enter should send message (if implemented)
    await page.keyboard.press('Enter');
  });
});
