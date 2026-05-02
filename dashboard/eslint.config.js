import js from '@eslint/js'
import globals from 'globals'
import reactHooks from 'eslint-plugin-react-hooks'
import reactRefresh from 'eslint-plugin-react-refresh'
import tseslint from '@typescript-eslint/eslint-plugin'
import tsParser from '@typescript-eslint/parser'
import { defineConfig, globalIgnores } from 'eslint/config'

const baseConfig = {
  extends: [
    js.configs.recommended,
    reactHooks.configs.flat.recommended,
    reactRefresh.configs.vite,
  ],
  languageOptions: {
    ecmaVersion: 2020,
    globals: {
      ...globals.browser,
      EncodedVideoChunk: 'readonly',
      ROSLIB: 'readonly',
      VideoDecoder: 'readonly',
      VideoDecoderConfig: 'readonly',
    },
    parserOptions: {
      ecmaVersion: 'latest',
      ecmaFeatures: { jsx: true },
      sourceType: 'module',
    },
  },
  rules: {
    'no-unused-vars': ['error', { varsIgnorePattern: '^[A-Z_]' }],
  },
}

export default defineConfig([
  globalIgnores(['dist', 'public/roslib.min.js', 'public/vendor/**']),
  {
    files: ['**/*.{js,jsx}'],
    ...baseConfig,
  },
  {
    files: ['**/*.{ts,tsx}'],
    ...baseConfig,
    languageOptions: {
      ...baseConfig.languageOptions,
      parser: tsParser,
    },
    plugins: {
      '@typescript-eslint': tseslint,
    },
    rules: {
      ...baseConfig.rules,
      '@typescript-eslint/no-unused-vars': ['error', { varsIgnorePattern: '^[A-Z_]' }],
      'no-unused-vars': 'off',
    },
  },
])
