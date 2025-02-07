# React Web Application <!-- omit from toc -->

## Table of Contents <!-- omit from toc -->

- [Introduction](#introduction)
- [Dependencies](#dependencies)
  - [Node.js](#nodejs)
  - [nvm](#nvm)
  - [npm](#npm)
  - [pnpm](#pnpm)
  - [Vite](#vite)
- [How to build and run the website locally](#how-to-build-and-run-the-website-locally)
- [Extra packages](#extra-packages)
- [Development environment](#development-environment)

## Introduction

This document contains information about the React application, how it was created and how to install it.

If you have `pnpm` installed, to download all dependencies required by the web application simply run the following commands:

```bash
pnpm install
pnpm build
pnpm dev
```

Otherwise, to install all required tools keep reading the following sections.

## Dependencies 

Following is list of all dependencies used by the React application.

### Node.js

Node.js is an open-source, cross-platform JavaScript runtime environment that enables developers to execute JavaScript code outside of a web browser. It provides the essential tools for modern JavaScript development, making it easy to manage dependencies for our React project.

### nvm

`nvm` (Node Version Manager) is a utility that simplifies the installation, management, and switching between different versions of Node.js. You can install nvm by following the instructions in this tutorial:

https://github.com/nvm-sh/nvm#install--update-script

Once installed, you can set up the LTS version of Node.js with the following commands:

```bash
nvm install --lts
nvm ls-remote --lts
nvm use --lts
```

### npm

`npm` (Node Package Manager) comes bundled with Node.js and is widely used to manage and distribute JavaScript packages. React and its related libraries are available through npm.

### pnpm

`pnpm` (Performant npm) is a fast and efficient package manager for JavaScript, offering an alternative to npm and yarn. It is used in this project to create, compile, and run the React application locally. To install pnpm, run the following command:

```bash
npm install --g pnpm
```

### Vite

Vite is a modern build tool and development server optimized for frontend frameworks like React. This project was initialized using Vite with the following command:

```bash
pnpm create vite . --template react-ts
```

## How to build and run the website locally

To install all required dependencies and run the website run the following commands:

```bash
pnpm install
pnpm build
pnpm dev
```

- **pnpm install:** This is for setting up the environment and downloading all required packages.

- **pnpm build:** This is only necessary when preparing for production deployment. During development, you generally don't need to run it unless you want to test a production build.

- **pnpm dev** This is sufficient for running your app in development mode after installing dependencies.

## Extra packages

The typical way to add additional packages to the React project is using the following command:

```bash
pnpm add <package>
```

Examples:

```bash
pnpm add three @types/three @react-three/fiber @react-three/drei
```

## Development environment

We use VSCode and Chrome to develop this web application.

**VSCode**

Install the following extensions in VSCode.

- [ESLint](https://marketplace.visualstudio.com/items?itemName=dbaeumer.vscode-eslint): this extension integrates ESLint into VS Code. To install ESLint run

  ```bash
  pnpm install -g eslint
  ```

- [Path Intellisense](https://marketplace.visualstudio.com/items?itemName=christian-kohler.path-intellisense): plugin to autocomplete filenames.

- [Prettier - Code formatter](https://marketplace.visualstudio.com/items?itemName=esbenp.prettier-vscode): it is an opinionated code formatter. It enforces a consistent style by parsing your code and re-printing it with its own rules that take the maximum line length into account, wrapping code when necessary. To install Prettier run:

  ```bash
  pnpm install -g prettier
  ```

- [React Import Sorter](https://marketplace.visualstudio.com/items?itemName=MrOnline.react-import-sorter): extension to sort imports.

**Chrome**

Install the following extensions in Chrome.

- [React Development Tools](https://chromewebstore.google.com/detail/fmkadmapgofadopljbjfkapdkoienihi): it adds React debugging tools to the Chrome Developer Tools.
