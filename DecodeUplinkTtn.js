function decodeUplink(input) {
  today=new Date();
  var data = {};
  var warn = [];
  var err = [];
  if(input.fPort == 2){
    warn.push("Utilizar el puerto 1");
  } else if(input.fPort == 3){
    err.push("No utilizar el puerto 3");
  } else {
    data.port = input.fPort;
    data.temperature = ((input.bytes[0] << 8) + input.bytes[1])/10;
    data.humidity = input.bytes[2];
    data.lighting = (input.bytes[3] << 8)+input.bytes[4];
    data.toxicGass = (input.bytes[5] << 8)+input.bytes[6];
    data.flammableGass = (input.bytes[7] << 8)+input.bytes[8];
    data.hour = String(today.getHours()+ 1) + ':' + String(today.getMinutes());
    data.date = String(today.getDate()) + '/' + String(today.getMonth()+ 1) + '/' + String(today.getFullYear());
  }
  return {
    data: data,
    warnings: warn,
    errors: err
  };
}