import { createRouter, createWebHistory } from 'vue-router';
import DigitalTwins from '../components/pages/digitalTwins.vue';
import Robots from '../components/pages/robots.vue';
import Develop from '../components/pages/develop.vue';

const routes = [
  {
    path: '/digitalTwins',
    name: 'DigitalTwins',
    component: DigitalTwins
  },
  {
    path: '/robots',
    name: 'Robots',
    component: Robots
  },
  {
    path: '/develop',
    name: 'Develop',
    component: Develop
  }
];

const router = createRouter({
  history: createWebHistory(),
  routes
});

export default router;