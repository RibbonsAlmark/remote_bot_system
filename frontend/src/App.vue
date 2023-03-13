<script setup>
import Menu from './components/menu.vue';
// import Header from './components/header.vue';
import Footer from './components/footer.vue';
import DigitalTwins from './components/pages/digitalTwins.vue';
import Robots from './components/pages/robots.vue';
import Develop from './components/pages/develop.vue';
import router from './router';
import {
  Box,
  MagicStick
} from '@element-plus/icons-vue'
</script>

<template>
  <div id="app" class="common-layout" style="height:100vh">
    <el-container style="height:100%">
      <el-header style="height:auto;padding:5px;background-color:#00000005;background">
        <el-row class="mb-4" style="float:right">
          <el-button round text=true>user</el-button>
          <el-button round @click="loginPanelVisible = true">login</el-button>
      </el-row>
      </el-header>
      <el-container>
        <el-aside width="200px">
          <Menu></Menu>
        </el-aside>
        <el-container>
          <el-main>
            <!-- <DigitalTwins></DigitalTwins> -->
            <!-- <Robots></Robots> -->
            <!-- <Develop></Develop> -->
            <router-view/>
          </el-main>
          <el-footer style="height:auto;padding:5px;background-color:#00000005;background">
            <Footer></Footer>
          </el-footer>
        </el-container>
      </el-container>
    </el-container>
    <el-dialog
        v-model="loginPanelVisible"
        title="登录"
        width="30%"
        align-center
        style="padding-left:30px;padding-right:30px;padding-top:30px;"
    >
      <el-form :model="loginForm" label-width="60px">
        <el-form-item label="用户名">
            <el-input v-model="loginForm.username" />
        </el-form-item>
        <el-form-item label="密码">
            <el-input v-model="loginForm.password" />
        </el-form-item>
      </el-form>
      <template #footer>
        <span class="dialog-footer">
            <el-button @click="loginPanelVisible = false">取消</el-button>
            <el-button type="primary" @click="login()">登录</el-button>
        </span>
      </template>
    </el-dialog> 
  </div>
</template>

<script>
import axios from 'axios'
import { reactive } from 'vue'

export default {
  name: 'App',
  router,
  data() {
    return {
      loginPanelVisible: false,
      loginForm: reactive({
          "username": "",
          "password": ""
      })
    }
  },
  methods: {
    login() {
      axios({
          method: "post",
          data: {
              "username": this.loginForm.username,
              "password": this.loginForm.password,
          },
          headers: {
              Authorization: ""
          },
          url: 'http://192.168.124.134:8000/login/'
      })
      .then(res => {
        localStorage.setItem('token', res.data["token"]);
        this.loginPanelVisible = false
      })
      .catch(error => {
          console.log(error)
      })
    }
  }
}
</script>
