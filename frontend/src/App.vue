<script setup>
import Menu from './components/menu.vue';
import Header from './components/header.vue';
import Footer from './components/footer.vue';
import {
  Box,
  MagicStick
} from '@element-plus/icons-vue'
</script>

<template>
  <div class="common-layout" style="height:100vh">
    <el-container style="height:100%">
      <el-header style="height:auto;padding:5px;background-color:#00000005;background">
        <Header></Header>
      </el-header>
      <el-container>
        <el-aside width="200px">
          <Menu></Menu>
        </el-aside>
        <el-container>
          <el-main>Main</el-main>
          <el-footer style="height:auto;padding:5px;background-color:#00000005;background">
            <Footer></Footer>
          </el-footer>
        </el-container>
      </el-container>
    </el-container>
  </div>
</template>