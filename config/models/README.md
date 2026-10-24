# Detector do LabSEA

`labsea-yolo26s-seg.pt` contém o checkpoint de segmentação já usado nos
ensaios sintéticos, com classes `chair` e `dining table`. O build público
usa esse arquivo para que o clone não dependa dos experimentos locais.
O arquivo tem aproximadamente 23 MB; não é um ponteiro Git LFS.

Confira sua integridade na raiz do repositório:

```bash
(cd config/models && sha256sum -c SHA256SUMS)
```

[O manifesto](labsea-yolo26s-seg.json) registra o SHA-256 e a procedência.
O peso é carregado pelo runtime Ultralytics; não há treinamento na partida.
Sua validação cobre somente o LabSEA controlado com imagens sintéticas.
O desempenho em imagens do robô físico ainda precisa ser avaliado.

Para trocar o modelo no build, use
`make sim-semantic-base-build SEMANTIC_WEIGHTS=/caminho/best.pt`.
O Dockerfile verifica a tarefa de segmentação e a ordem das duas classes.
