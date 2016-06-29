void enviarString (String cadena)
{
  int cadena_len = cadena.length() + 1;
  char char_array[cadena_len];
  cadena.toCharArray(char_array, cadena_len);
  send(char_array);
}

void send (char *message)
{
  vw_send((uint8_t *)message, strlen(message));
  vw_wait_tx(); // Wait until the whole message is gone
  
}
