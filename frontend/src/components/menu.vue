<template>
  <el-menu
    default-active="1"
    class="el-menu-vertical-demo"
    style="height:100%"
    @open="handleOpen"
    @close="handleClose"
  >
    <el-menu-item index="1">
      <el-icon><EcosystemIcon /></el-icon>
      <span>场地数字孪生</span>
    </el-menu-item>
    <el-menu-item index="2">
      <el-icon><Box /></el-icon>
      <span>查看机器人列表</span>
    </el-menu-item>
    <el-menu-item index="3">
      <el-icon><MagicStick /></el-icon>
      <span>机器人远程开发</span>
    </el-menu-item>
  </el-menu>
</template>

<script lang="ts" setup>
import EcosystemIcon from './icons/IconEcosystem.vue'
import {
  Box,
  MagicStick
} from '@element-plus/icons-vue'
const handleOpen = (key: string, keyPath: string[]) => {
  console.log(key, keyPath)
}
const handleClose = (key: string, keyPath: string[]) => {
  console.log(key, keyPath)
}
</script>