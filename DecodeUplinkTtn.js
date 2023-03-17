function decodeUplink(input) {
  today=new Date();
  var datos = {};
  var warn = [];
  var err = [];
  if(input.fPort == 2){
    warn.push("Utilizar el puerto 1");
  } else if(input.fPort == 3){
    err.push("No utilizar el puerto 3");
  } else {
    datos.port = input.fPort;
    datos.temperatura = ((input.bytes[0] << 8) + input.bytes[1])/10;
    datos.humedad = input.bytes[2];
    datos.luz = (input.bytes[3] << 8)+input.bytes[4];
    datos.nocivos = (input.bytes[5] << 8)+input.bytes[6];
    datos.inflamable = (input.bytes[7] << 8)+input.bytes[8];
    datos.hora = String(today.getHours()+ 1) + ':' + String(today.getMinutes());
    datos.fecha = String(today.getDate()) + '/' + String(today.getMonth()+ 1) + '/' + String(today.getFullYear());
  }
  return {
    data: datos,
    warnings: warn,
    errors: err
  };
}