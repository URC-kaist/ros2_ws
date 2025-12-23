# React + Vite

This template provides a minimal setup to get React working in Vite with HMR and some ESLint rules.

Currently, two official plugins are available:

- [@vitejs/plugin-react](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react) uses [Babel](https://babeljs.io/) (or [oxc](https://oxc.rs) when used in [rolldown-vite](https://vite.dev/guide/rolldown)) for Fast Refresh
- [@vitejs/plugin-react-swc](https://github.com/vitejs/vite-plugin-react/blob/main/packages/plugin-react-swc) uses [SWC](https://swc.rs/) for Fast Refresh

## React Compiler

The React Compiler is not enabled on this template because of its impact on dev & build performances. To add it, see [this documentation](https://react.dev/learn/react-compiler/installation).

## Expanding the ESLint configuration

If you are developing a production application, we recommend using TypeScript with type-aware lint rules enabled. Check out the [TS template](https://github.com/vitejs/vite/tree/main/packages/create-vite/template-react-ts) for information on how to integrate TypeScript and [`typescript-eslint`](https://typescript-eslint.io) in your project.

## Hosting with nginx (same domain as SiK gateway)

1) Build and deploy the dashboard:

```bash
./scripts/deploy_dashboard.bash
```

2) Install nginx and enable the site config:

```bash
sudo apt update
sudo apt install nginx
sudo cp ./scripts/nginx/mr2-dashboard.conf /etc/nginx/sites-available/mr2
sudo ln -s /etc/nginx/sites-available/mr2 /etc/nginx/sites-enabled/mr2
sudo nginx -t
sudo systemctl reload nginx
```

By default, the dashboard is served from `/var/www/mr2-dashboard` and the WebSocket proxy is `/sik-ws` to `http://127.0.0.1:8081`. Adjust `server_name` in `./scripts/nginx/mr2-dashboard.conf` as needed.
