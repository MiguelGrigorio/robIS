# Calibrações das câmeras fixas do LabSEA

Os arquivos `calib_rt1.npz` a `calib_rt4.npz` foram fornecidos pelo usuário.
Eles correspondem às quatro câmeras de teto representadas no GLB; não são
a calibração da câmera USB do robô e não são carregados pelo launch físico.

Cada arquivo contém `cam_id`, `width=1280`, `height=720`, `K`, `nK`, `dist`,
`roi`, `rt`, `rt_inv`, `P_original`, `P_undistorted` e `escala`.
`dist` possui 14 coeficientes: não deve ser truncado para caber em um modelo
ROS de cinco/oito coeficientes. O uso futuro deve preservar esse modelo de
distorção ou trabalhar com imagens corrigidas e os intrínsecos correspondentes.
O [replay das câmeras fixas](../../docs/fixed-cameras.md) já suporta ambos os
casos, preservando os 14 coeficientes nas imagens brutas.

As translações de `rt_inv` coincidem com os modelos do GLB após a conversão
de eixos documentada pelo importador:

| ID | X | Y | Z |
|---|---:|---:|---:|
| 1 | 2,736790 | 3,698505 | 2,867640 |
| 2 | −2,878442 | 3,732167 | 2,850206 |
| 3 | −2,836204 | −3,776548 | 2,885130 |
| 4 | 2,826685 | −3,808250 | 2,842501 |

Essa correspondência identifica o referencial da reconstrução. Antes de
alimentar a memória semântica real, ainda será necessário alinhar esse
referencial com o `map` do robô e confirmar a configuração dos streams
(resolução, correção de distorção e recorte). O replay foi implementado e
testado com imagens sintéticas do Gazebo. Ainda não foram recebidas imagens
das câmeras físicas; o alinhamento com o mapa real continua pendente.
