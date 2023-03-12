<template>
    <div style="height:100%;width:100%;background-color:black">
        <div style="height:auto;width:100%;background-color:red;padding:5px">
            <el-table :data="robotsTableData" style="width: 100%">
                <el-table-column prop="name" label="名称" width="180" />
                <el-table-column prop="type" label="类型" width="140" />
                <el-table-column prop="uuid" label="UUID" min-width="400" />
                <el-table-column prop="ip" label="IP" width="180" />
                <el-table-column prop="online" label="在线" width="100" />
                <el-table-column prop="allocated" label="已分配" width="100" />
                <el-table-column fixed="right" label="操作" width="120" >
                    <template #default="scope">
                        <el-button link type="primary" size="small" @click="onAcquireCardOpen(scope.row)" :disabled="scope.row.allocated">申请</el-button>
                        <el-button link type="primary" size="small" disabled>编辑</el-button>
                    </template>
                </el-table-column>
            </el-table>
        </div>
        <el-dialog
            v-model="centerDialogVisible"
            title="申请机器人资源"
            width="55%"
            align-center
            style="padding-left:30px;padding-right:30px;padding-top:30px;"
        >
            <el-form :model="acquireForm" label-width="160px">
                <el-form-item label="机器人UUID">
                    <el-input v-model="acquireForm.robot_uuid" disabled />
                </el-form-item>
                <el-form-item label="机器人用户名">
                    <el-select v-model="acquireForm.robot_username" placeholder="选择机器人用户名">
                        <el-option v-for="(ftpInfo, index) in ftpTableData" :key="index" :label="ftpInfo.username" :value="ftpInfo.username" />
                    </el-select>
                </el-form-item>
                <el-form-item label="vscode-web 密码">
                    <el-input v-model="acquireForm.password" />
                </el-form-item>
                <el-form-item label="vscode-web 工作空间">
                    <el-input v-model="acquireForm.workspace" />
                </el-form-item>
            </el-form>
            <template #footer>
                <span class="dialog-footer">
                    <el-button @click="centerDialogVisible = false">取消</el-button>
                    <el-button type="primary" @click="acquireRobot()">提交</el-button>
                </span>
            </template>
        </el-dialog>
    </div>
</template>

<script>
import axios from 'axios'
import { reactive } from 'vue'


export default {
    data() {
        return {
            robotsTableData: [], 
            ftpTableData: [], 
            centerDialogVisible: false,
            acquireCardVisible: false,
            acquireForm: reactive({
                "robot_uuid": "",
                "robot_username": "",
                "password": "",
                "workspace": ""
            })
        }
    },
    mounted() {
        axios.get('http://192.168.124.134:8000/robot/all')
          .then(res => {
                this.robotsTableData = res.data
            })
            .catch(error => {
                console.log(error)
            })
    },
    methods: {
        onAcquireCardOpen(row) {
            let robotUuid = row.uuid
            axios({
                method: "get",
                params: {"robot_uuid": robotUuid},
                url: 'http://192.168.124.134:8000/ftp_info/'
            })
            .then(res => {
                this.ftpTableData = res.data
                console.log(this.ftpTableData)
            })
            .catch(error => {
                console.log(error)
            })
            this.acquireForm.robot_uuid = robotUuid
            this.centerDialogVisible = true
        },
        acquireRobot() {
            console.log(this.acquireForm)
            axios({
                method: "post",
                data: {
                    "robot_uuid": this.acquireForm.robot_uuid,
                    "robot_username": this.acquireForm.robot_username,
                    "password": this.acquireForm.password,
                    "workspace": this.acquireForm.workspace,
                },
                headers: {
                    Authorization: ""
                },
                url: 'http://192.168.124.134:8000/user/acquire_robot/'
            })
            axios.post('http://192.168.124.134:8000/user/acquire_robot/', {
                "robot_uuid": this.acquireForm.robot_uuid,
                "robot_username": this.acquireForm.robot_uuid,
                "password": this.acquireForm.robot_uuid,
                "workspace": this.acquireForm.robot_uuid,
            })
            .then(res => {
                console.log(res)
            })
            .catch(error => {
                console.log(error)
            })
            centerDialogVisible = false
        }
    }
}
</script>

<style scoped>
.dialog-footer button:first-child {
  margin-right: 10px;
}
</style>