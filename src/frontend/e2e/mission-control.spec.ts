import {test,expect} from "@playwright/test";
import {instalarMocksApi} from "./support/api-mocks";
const robot=(extra:Record<string,unknown>={})=>({firmware:"maze",boot_id:7,remote_enabled:true,state:"ready",width:4,corner:0,heading:1,received_at_ms:Date.now(),...extra});
test("diagnóstico bloqueia partida e parada aguarda confirmação real",async({page})=>{
 await instalarMocksApi(page,{sessoes:[]});
 await page.routeWebSocket('ws://localhost:3000/ws?role=dashboard',()=>{});
 await page.route('**/api/robot/status',route=>route.fulfill({json:{connected:true,robot:robot({firmware:"diagnostic",remote_enabled:false})}}));
 const commands:unknown[]=[];
 await page.route('**/api/robot/command',async route=>{
  commands.push(route.request().postDataJSON());
  await new Promise(resolve=>setTimeout(resolve,300));
  await route.fulfill({json:{accepted:true,message:"Motores desabilitados"}});
 });
 await page.goto('/');
 await expect(page.getByRole('button',{name:'Iniciar missão'})).toBeDisabled();
 await expect(page.getByText('A ESP está com o teste Bluetooth.',{exact:false})).toBeVisible();
 await page.getByRole('button',{name:'Parar robô'}).click();
 await expect(page.getByRole('status',{name:'Resposta do comando'})).toHaveText('Solicitando parada à ESP…');
 await expect(page.getByRole('status',{name:'Resposta do comando'})).toContainText('ESP confirmou: Motores desabilitados');
 expect(commands).toEqual([{action:'stop'}]);
});
test("executa 4x4 e depois 8x4 sem iniciar ao mudar seleção ou reconectar",async({page})=>{
 await instalarMocksApi(page,{sessoes:[]});
 let current=robot();const commands:Array<Record<string,number|string>>=[];
 let send:(p:unknown)=>void=()=>{};
 await page.routeWebSocket('ws://localhost:3000/ws?role=dashboard',socket=>{send=p=>socket.send(JSON.stringify(p));});
 await page.route('**/api/robot/status',route=>route.fulfill({json:{connected:true,robot:current}}));
 await page.route('**/api/robot/command',route=>{
  const command=route.request().postDataJSON();commands.push(command);
  current=robot({state:'armed',width:command.width});
  send({type:'maze',width:command.width,height:4});
  return route.fulfill({json:{accepted:true,message:'Missão armada; partida em 5 segundos'}});
 });
 await page.goto('/');
 await expect(page.getByRole('button',{name:'Iniciar missão'})).toBeEnabled();
 expect(commands).toHaveLength(0);
 await page.getByRole('button',{name:'Iniciar missão'}).click();
 await expect(page.getByRole('status',{name:'Resposta do comando'})).toContainText('ESP confirmou');
 await expect(page.getByRole('button',{name:'Iniciar missão'})).toBeDisabled();
 await expect(page.getByRole('heading',{name:'Mapa do labirinto (4x4)'})).toBeVisible();
 current=robot({state:'completed'});
 await expect(page.getByTestId('mission-state')).toHaveText('Concluído');
 await page.getByLabel('Labirinto da missão').selectOption('8');
 expect(commands).toHaveLength(1);
 await expect(page.getByRole('heading',{name:'Mapa do labirinto (4x4)'})).toBeVisible();
 await page.getByRole('button',{name:'Iniciar missão'}).click();
 await expect(page.getByRole('heading',{name:'Mapa do labirinto (8x4)'})).toBeVisible();
 expect(commands).toEqual([{action:'start',width:4,corner:0,heading:1},{action:'start',width:8,corner:0,heading:1}]);
 await page.getByRole('button',{name:'Reconectar painel'}).click();
 expect(commands).toHaveLength(2);
});
test("não anuncia início se a ESP recusar o comando",async({page})=>{
 await instalarMocksApi(page,{sessoes:[]});
 await page.routeWebSocket('ws://localhost:3000/ws?role=dashboard',()=>{});
 await page.route('**/api/robot/status',route=>route.fulfill({json:{connected:true,robot:robot()}}));
 await page.route('**/api/robot/command',route=>route.fulfill({status:409,json:{accepted:false,message:'Sensores indisponíveis'}}));
 await page.goto('/');
 await page.getByRole('button',{name:'Iniciar missão'}).click();
 await expect(page.getByRole('alert')).toContainText('Sensores indisponíveis');
 await expect(page.getByTestId('mission-state')).toHaveText('Pronto');
});

test("mostra freio por eco, mantém parada disponível e depois mostra retomada",async({page})=>{
 await instalarMocksApi(page,{sessoes:[]});
 await page.routeWebSocket('ws://localhost:3000/ws?role=dashboard',()=>{});
 let recovering=true;
 await page.route('**/api/robot/status',route=>route.fulfill({json:{connected:true,robot:robot({state:'running',recovering_front_echo:recovering})}}));
 await page.goto('/');
 await expect(page.getByTestId('mission-state')).toHaveText('Freado · aguardando eco frontal');
 await expect(page.getByRole('button',{name:'Iniciar missão'})).toBeDisabled();
 await expect(page.getByRole('button',{name:'Parar robô'})).toBeEnabled();
 recovering=false;
 await expect(page.getByTestId('mission-state')).toHaveText('Explorando o labirinto');
});
