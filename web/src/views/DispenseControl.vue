<template>
  <div class="dispense-view">
    <FeedbackDispensePanel />
  </div>
</template>

<script setup>
// 2026-10-26 融合改造：这一页只保留「三阶段闭环加粉」。
// 原来的「恒速加粉」(ConstantRatePanel) 走的是老后端的 /constant-rate/start
// 与窗口位置模型，目前后端没有这套实现（推杆整段锁死、只靠振幅调速率），
// 留着按钮只会点出 404。等以后确实需要恒速标定时再单独接。
import FeedbackDispensePanel from '../components/FeedbackDispensePanel.vue'
</script>

<style scoped>
.dispense-view { display: grid; gap: 20px; min-width: 0; }
</style>
