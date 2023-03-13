<template>
  <div style="height:100%">
    <iframe :src="externalUrl" width="100%" height="100%"></iframe>
  </div>
</template>

<script>
import { ref } from 'vue'

export default {
  setup() {
    const externalUrl = ref("http://192.168.124.134:8080")

    return {
      externalUrl
    }
  }
}
</script>