# robis_description

Contém a descrição física do robô: URDF/Xacro, malhas e dimensões.

- `config/robot_parameters.yaml`: medidas e nomes dos referenciais.
- `urdf/robis.urdf.xacro`: modelo usado pelo robô e pelo Gazebo.
- `meshes/`: aparência do robô.

O perfil básico usa esse modelo para publicar as TFs fixas dos sensores.
A odometria publica a transformação de `odom` para `base_footprint`.
O Gazebo fornece os estados das rodas e dos casters em `/joint_states`.
Essa publicação das juntas móveis ainda está pendente no driver físico.

Os casters possuem pivô vertical e roda com rolamento passivo. As massas
totais e os pontos de montagem foram preservados; os novos parâmetros de
suporte, deslocamento do eixo, amortecimento e atrito são aproximações no YAML.
As dimensões, massas e extrínsecas ainda exigem comparação com medições do
robô. Veja os [testes e limites do modelo](../../docs/simulation.md).
Veja o [README principal](../../README.md).

O visual atual usa [o CAD fornecido](meshes/updated/README.md), com as peças
estáticas separadas das rodas e casters articulados. As configurações físicas
foram mantidas. Os arquivos Blender e modelos anteriores ficam em `extras/assets/`.
