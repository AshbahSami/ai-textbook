// temp-docusaurus-site/tests/auth.spec.ts
import { test, expect } from '@playwright/test';

// Define the base URL for your Docusaurus app (usually localhost:3000 during development)
const BASE_URL = 'http://localhost:3000'; // Assuming Docusaurus runs on port 3000
const BACKEND_API_URL = 'http://localhost:3001'; // Assuming backend runs on port 3001

test.beforeEach(async ({ page }) => {
  // Clear cookies before each test to ensure a clean state
  await page.goto(BASE_URL);
  await page.context().clearCookies();
  // Ensure the backend is running and Better-Auth service is mocked or available
  // For E2E, we assume the backend is operational.
});

test.describe('Authentication Flow', () => {
  test('should allow a new user to sign up and then sign in', async ({ page }) => {
    await page.goto(`${BASE_URL}/signup`);

    // Generate unique credentials for each test run
    const timestamp = Date.now();
    const email = `testuser_${timestamp}@example.com`;
    const password = `Password${timestamp}!`;

    // Fill out the signup form
    await page.fill('#email', email);
    await page.fill('#password', password);
    await page.fill('#confirm-password', password);

    // Fill out software background
    await page.selectOption('#primaryLanguages', ['Python', 'JavaScript/TypeScript']);
    await page.selectOption('#cloudFamiliarity', 'Intermediate');
    await page.selectOption('#preferredOs', 'Linux');
    await page.selectOption('#databaseExperience', ['SQL']);

    // Fill out hardware background
    await page.selectOption('#embeddedSystemsExperience', 'No');
    await page.selectOption('#gpuFamiliarity', 'Basic');

    await page.click('button[type="submit"]');

    // Expect a success message after signup
    await expect(page.locator('.alert--success')).toContainText('Account created successfully! You can now log in.');

    // --- Now, try to sign in with the new user ---
    await page.goto(`${BASE_URL}/signin`);

    await page.fill('#email', email);
    await page.fill('#password', password);
    await page.click('button[type="submit"]');

    // Expect successful login and redirection
    await expect(page).toHaveURL(/.*personalized-dashboard/); // Redirects to personalized dashboard
    await expect(page.locator('.alert--success')).toContainText('Login successful'); // Assuming success message is displayed
  });

  test('should prevent signup with mismatched passwords', async ({ page }) => {
    await page.goto(`${BASE_URL}/signup`);

    const email = 'mismatch@example.com';
    const password = 'Password123!';
    const confirmPassword = 'DifferentPassword123!';

    await page.fill('#email', email);
    await page.fill('#password', password);
    await page.fill('#confirm-password', confirmPassword); // Mismatched password

    // Fill out required background data to bypass other validations
    await page.selectOption('#primaryLanguages', 'Python');
    await page.selectOption('#cloudFamiliarity', 'None');
    await page.selectOption('#preferredOs', 'Linux');
    await page.selectOption('#databaseExperience', 'SQL');
    await page.selectOption('#embeddedSystemsExperience', 'No');
    await page.selectOption('#gpuFamiliarity', 'None');

    await page.click('button[type="submit"]');

    // Expect an error message for mismatched passwords
    await expect(page.locator('.alert--danger')).toContainText('Passwords do not match');
    // Ensure no redirection or success message
    await expect(page).toHaveURL(/.*signup/);
    await expect(page.locator('.alert--success')).not.toBeVisible();
  });

  test('should display error for invalid login credentials', async ({ page }) => {
    await page.goto(`${BASE_URL}/signin`);

    await page.fill('#email', 'nonexistent@example.com');
    await page.fill('#password', 'wrongpassword');
    await page.click('button[type="submit"]');

    // Expect an error message for invalid credentials
    await expect(page.locator('.alert--danger')).toContainText('Failed to sign in. Please check your credentials and try again.');
    await expect(page).toHaveURL(/.*signin/); // Should remain on signin page
  });

  // Test personalized content display
  test('should display personalized content for a logged-in user', async ({ page }) => {
    // First, log in a user (assuming a user already exists for simplicity in this specific test)
    // In a real scenario, you might have a test user pre-registered or mock the login
    await page.goto(`${BASE_URL}/signin`);
    await page.fill('#email', 'testuser_for_personalization@example.com'); // Use a known user
    await page.fill('#password', 'Password123!');
    await page.click('button[type="submit"]');

    await expect(page).toHaveURL(/.*personalized-dashboard/);

    // Navigate to the home page where PersonalizedContent is rendered
    await page.goto(BASE_URL);

    // Expect PersonalizedContent component to be visible
    await expect(page.locator('.card:has-text("Your Personalized Profile")')).toBeVisible();
    // Expect some personalized data to be displayed (e.g., from the mock user)
    await expect(page.locator('.card:has-text("Primary Languages: Python")')).toBeVisible();
  });
});
