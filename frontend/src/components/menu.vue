<template>
  <el-menu
    default-active="1"
    class="el-menu-vertical-demo"
    style="height:100%"
  >
    <el-menu-item index="1">
      <el-icon><EcosystemIcon /></el-icon>
      <router-link to="/digitalTwins" style="color:black;width:100%">场地数字孪生</router-link>
    </el-menu-item>
    <el-menu-item index="2">
      <el-icon><Box /></el-icon>
      <router-link to="/robots" style="color:black;width:100%">查看机器人列表</router-link>
    </el-menu-item>
    <el-menu-item index="3">
      <el-icon><MagicStick /></el-icon>
      <router-link to="/develop" style="color:black;width:100%">机器人远程开发</router-link>
    </el-menu-item>
    <el-menu-item index="4">
      <el-icon><Soccer /></el-icon>
      <router-link to="/rviz" style="color:black;width:100%">Rviz Web</router-link>
    </el-menu-item>
  </el-menu>
</template>

<script lang="ts" setup>
import EcosystemIcon from './icons/IconEcosystem.vue'
import router from '../router';
import {
  Box,
  MagicStick,
  Soccer
} from '@element-plus/icons-vue'
const handleClick = (key: string, keyPath: string[]) => {
  console.log(key, keyPath)
  router.push("robots")
}
// export default {
//   name: "Menu",
//   router,
//   methods: {
//     handleOpen(key: string, keyPath: string[]) {
//       console.log(key, keyPath)
//     },
//     handleClose (key: string, keyPath: string[]) {
//       console.log(key, keyPath)
//     }
//   },
// }
</script>