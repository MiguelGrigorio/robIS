# robis_interfaces

Define as mensagens de comunicação específicas do projeto:

- `GotoObject.action`: pedido, resultado e progresso de uma busca por objeto.
- `DetectObject.srv`: consulta de detecção visual.
- `ExternalObject.msg`: detecção espacial externa, revisão e sensibilidade da projeção.
- `ExternalObjects.msg`: conjunto de observações externas, com origem,
  ambiente, hash do mapa e prazo de validade.

As mensagens externas permitem que CCTVs ou outros serviços alimentem a
memória semântica. Cada detecção conserva seu timestamp original; publicar
novamente uma posição antiga não a transforma em uma observação nova.
Veja o [contrato das câmeras fixas](../../docs/fixed-cameras.md).

As interfaces são geradas durante o build. O pacote é incluído no perfil
básico porque o pacote de visão compartilha esses contratos, embora a busca
semântica não seja iniciada nessa etapa.

Veja o [README principal](../../README.md).
