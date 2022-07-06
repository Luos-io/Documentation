/** @type {import('eslint').Linter.Config} */
module.exports = {
	root: true,
	parser: '@typescript-eslint/parser',
	plugins: ['@typescript-eslint'],
	extends: [
		'eslint:recommended',
		'plugin:@typescript-eslint/recommended',
		'plugin:mdx/recommended',
		'prettier',
	],
	settings: {
		'mdx/code-blocks': true,
		'mdx/language-mapper': {},
	},
};
