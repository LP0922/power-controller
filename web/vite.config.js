import { defineConfig } from 'vite'
import vue from '@vitejs/plugin-vue'

export default defineConfig({
  plugins: [vue()],
  server: {
    // 固定绑 IPv4：后端 app.py 只监听 127.0.0.1:8765，
    // 而 Vite 默认会绑到 ::1，代理出站时按 IPv6 解析会直接"连接被拒"（2026-10-26 实测）。
    host: '127.0.0.1',
    port: 5173,
    proxy: {
      '/api': {
        target: 'http://127.0.0.1:8765',
        changeOrigin: true,
      },
    },
  },
})
