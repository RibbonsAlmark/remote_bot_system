<template>
    <div style="height:100%;width:100%;">
        <div style="height:100%;width:100%;padding:15px">
            <div style="height:100px;width:100%;padding:5px"></div>
            <div style="height:auto;width:100%;background-color:#edecec5c;padding:15px">
                <el-table :data="myRobotsTableData" style="width: 100%">
                    <el-table-column prop="robot_name" label="机器人名称" min-width="120" />
                    <el-table-column prop="robot_username" label="机器人用户名" min-width="120" />
                    <el-table-column prop="code_server_url" label="vscode-web URL" min-width="180">
                        <template #default="scope">
                            <!-- <a href="scope.row.code_server_url" target="_blank">{{scope.row.code_server_url}}</a> -->
                            <el-link href="scope.row.code_server_url" target="_blank">{{scope.row.code_server_url}}</el-link>
                        </template>
                    </el-table-column>
                    <el-table-column prop="code_server_workspace" label="vscode-web工作目录" min-width="200" />
                    <el-table-column prop="robot_workspace" label="机器人工作目录" min-width="200"  />
                    <el-table-column prop="robot_type" label="机器人类型" min-width="120" />
                    <el-table-column prop="robot_ip" label="机器人IP" min-width="180" />
                    <el-table-column prop="robot_uuid" label="机器人UUID" min-width="300" />
                    <el-table-column fixed="right" label="操作" width="60" >
                        <template #default="scope">
                            <el-button link type="primary" size="small" @click="releaseRobot(scope.row)" :disabled="scope.row.allocated">释放</el-button>
                        </template>
                    </el-table-column>
                </el-table>
            </div>
        </div>
    </div>
</template>

<script>
import axios from 'axios'
import { reactive } from 'vue'


export default {
    data() {
        return {
            myRobotsTableData: [], 
            // ftpTableData: [], 
            // centerDialogVisible: false,
            // acquireCardVisible: false,
            // acquireForm: reactive({
            //     "robot_uuid": "",
            //     "robot_username": "",
            //     "password": "",
            //     "workspace": ""
            // })
        }
    },
    mounted() {
            axios({
                method: "get",
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: 'http://192.168.124.134:8000/robot/get_my_robots'
            })
            .then(res => {
                this.myRobotsTableData = res.data
            })
            .catch(error => {
                console.log(error)
            })
    },
    methods: {
        releaseRobot(row) {
            axios({
                method: "post",
                data: {
                    "robot_uuid": row.robot_uuid,
                    "robot_username": row.robot_username,
                },
                headers: {
                    Authorization: localStorage.getItem('token')
                },
                url: 'http://192.168.124.134:8000/robot/release'
            })
            .then(res => {
                console.log(res)
            })
            .catch(error => {
                console.log(error)
            })
        }
    }
}
</script>

<style scoped>
.dialog-footer button:first-child {
  margin-right: 10px;
}
</style>