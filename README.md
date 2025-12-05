# INF01047-Trabalho-Final

Um jogo FPS simples onde o objetivo do personagem é arremessar projéteis em pássaros inimigos no menor tempo e menor número de queda das plataformas possível.

## Imagens de Exemplo
![Ex1](./misc/ex1.png)
![Ex2](./misc/ex2.png)


## Contribuição de cada membro
- Estevan:
  - Movimentação dos pássaros por meio de curvas de bézier cúbicas piecewise
  - Instanciação dos projéteis
  - Colisão projéteis com plataformas (Colisão Esfera x AABB)
  - Colisão projéteis com pássaros (Colisão Esfera x Esfera)
  - Texturização pássaros, letreiros, projétil e plataformas
  - Letreiros FCG (objeto com iluminação Gouraud)
- Eduardo:
  - Desenho e movimentação do personagem
  - Câmera livre e look-at
  - Teste de colisão cubo (OBB) x cubo (OBB), usada entre persogem e plataformas e persongem e os letreiros da cena
  - Teste de colisão raio x cubo (AABB), usada para câmera não atravessar plataformas
  - Texturização personagem e plataformas
  - Skybox

A atribuição dos itens aos nomes não significa que os os itens atribuídos a uma pessoa foram exclusivamente desenvolvidos por ela, apenas que foi *principalmente* desenvolvido por ela.
    
## Lista de cumprimento dos requisitos
| Requisito | Onde Foi Cumprido |
| :--- | :--- |
| Malhas Poligonais Complexas | Modelo do personagem e dos pássaros |
| Transformações Geométricas Controlodas pelo Usuário | Controle do movimento do personagem, incluindo movimentação no plano xz, pulo e rotação |
| Câmera Livre e Câmera Look-At | Uma câmera livre que se movimenta junto ao personagem e rotaciona numa esfera à sua volta. Câmera look-at para visão em terceira pessoa do personagem de frente (apertando a tecla T) e de costas (apertando a tecla Y)|
| Instâncias de Objetos | Instâncias de pássaros, plataformas, projéteis e letreiros |
| 3 Tipos de Intersecção | 1. cubo (OBB) x cubo (OBB) para intersecção entre personagem e plataformas e personagem e letreiros; 2. raio x cubo (AABB) para câmera e plataformas; 3. esfera x esfera para pássaros e projéteis; 4. esfera x cubo (AABB) para projéteis e plataformas |
| Modelos de Iluminação Difusa e Blinn-Phong |  Difusa: personagem; Phong: todos os outros objetos na cena |
| Modelos de Interpolação de Phong e Gouraud | Modelo de interpolação de Gouraud: um dos letreiros "FCG" na cena. Modelo de interpolação de Phong: todo o restante dos objetos na cena |
| Mapeamento de Texturas em Todos os Objetos | Textura do personagem e projéteis mapeada com coordenadas definidas no modelo obj, textura dos pássaros feita com mapeamento esférico e textura das plataformas e letreiros feita com mapeamento planar  |
| Movimento com Com Curva Bézier Cúbica | Movimentação dos pássaros na cena é definida por meio de curvas de Bézier cúbicas |
| Animações Baseadas no Tempo | Tudo o que se movimenta na cena utiliza o mesmo tempo/delta para se mover |

## Controles
![Controles do Jogo](./misc/controles.png)


## Discussão sobre IA Generativa
Utilizamos IA Generativa no trabalho para a implementação de algumas funções específicas, para tirar dúvidas e tentando corrigir problemas na nossa implementação. Especificamente:
1. Utilizamos código gerado por IA Generativa (Gemini) *diretamente (copia e cola)* para o cálculo de derivadas ao montar as curvas de Bézier.
2. Utilizamos IA Generativa (Gemini e ChatGPT) para tirar dúvidas teóricas, principalmente a respeito do skybox, do movimento com curvas de Bézier e de colisões, eventualmente pegando partes de código de exemplo gerados por essas IAs e adaptando ao nosso código.
3. Diversas vezes utilizamos IA Generativa como forma de identificar problemas na nossa implementação.

O que observamos foi que a IA Generativa é muito útil para o caso (2): tirar dúvidas sobre um tópico já muito bem estabelecido. Para o caso (1), percebemos que dificilmente o código para funções complexas gerado por IAs é utilizável de forma direta, normalmente sendo possível apenas em casos em que o procedimento seja muito padrão, como o caso da aplicação de fórmulas. E para o caso (3), sentimos que a IA muitas vezes mais atrapalhou que ajudou. Em parte isso se deve ao nosso código muito extenso com grau de acoplamento muito alto.


