import {test,expect} from '@playwright/test';
test.beforeEach(async({page})=>{
 await page.route('**/api/robot/status',route=>route.fulfill({json:{connected:false,robot:null}}));
});
const packet=(extra:Record<string,unknown>={})=>({type:'bt_telemetry_test',seq:37,uptime_ms:9000,heap_bytes:81920,
 motors_enabled:false,ultrasonic_enabled:false,received_at_ms:Date.now(),
 imu:{valid:true,yaw_deg:12.5,gyro_z_dps:0.06,accel_g:{x:0.01,y:0.02,z:1}},...extra});
test('mostra IMU Bluetooth, distingue ausência de atualização e não inventa corrida',async({page})=>{
 const historyRequests:string[]=[];page.on('request',request=>{if(request.url().includes('/api/sessions'))historyRequests.push(request.url());});
 await page.routeWebSocket('ws://localhost:3000/ws?role=dashboard',socket=>{
  socket.send(JSON.stringify({type:'bt_link_status',status:'connected',received_at_ms:Date.now()}));socket.send(JSON.stringify(packet()));
 });
 await page.goto('/?fonte=bluetooth');
 await expect(page.getByRole('heading',{name:'Bluetooth ao vivo',exact:true})).toBeVisible();
 await expect(page.getByTestId('bluetooth-yaw')).toHaveText('12.500 °');
 await expect(page.getByTestId('bluetooth-sequence')).toHaveText('37');
 await expect(page.getByRole('status')).toHaveText('Recebendo dados reais');
 await expect(page.getByRole('heading',{name:/Mapa do labirinto/})).toBeVisible();
 await expect(page.getByRole('heading',{name:'Status e telemetria',exact:true})).toBeVisible();
 await expect(page.getByRole('button',{name:'Histórico e diagnósticos'})).toBeVisible();
 expect(historyRequests).toHaveLength(0);
 await expect(page.getByRole('status')).toHaveText('Sem dados recentes',{timeout:5000});
});
test('replay antigo não aparece como leitura nova e IMU inválida não vira zero',async({page})=>{
 await page.routeWebSocket('ws://localhost:3000/ws?role=dashboard',socket=>{
  socket.send(JSON.stringify({type:'bt_link_status',status:'connected',received_at_ms:Date.now()}));
  socket.send(JSON.stringify(packet({received_at_ms:Date.now()-60000,imu:{valid:false,yaw_deg:null,gyro_z_dps:null,accel_g:{x:null,y:null,z:null}}})));
 });
 await page.goto('/?fonte=bluetooth');
 await expect(page.getByRole('status')).toHaveText('Sem dados recentes');
 await expect(page.getByTestId('bluetooth-yaw')).toHaveText('Sem leitura');
 await expect(page.getByRole('alert')).toContainText('IMU não forneceu uma leitura válida');
});
