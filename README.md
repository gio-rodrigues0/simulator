# Ponderada 2 - Módulo 6

1. Definição da diferença aproximação máxima para considerar o Pose como certo.
2. Criação da classe MissionControl para ler um arquivo csv com os pontos que devem ser passados pelo robô e a criação de uma fila e seus métodos enqueue e dequeue.
3. Criação da classe Pose com alguns métodos que evitam repetição de código desnecessária.
4. Criação da classe SimulatorControler, que é o nosso controlador, que defini uma posição inicial para a nossa posição atual e a nossa posição meta, cria o nosso publisher e o nosso subscriber, cria um timer, dois métodos de callback e um método para atualizar a nossa posição meta.
5. Criação de um callback que verifica se a primeira posição já foi definida, faz as diferenças entre a posição atual e a meta, estabelecendo se a aproximação já é boa o suficiente para parar a movimentação e sair quando a fila estiver vazia.
6. Criação do método de atualizar a posição meta, acrescendo o valor da primeira posição da fila na posição atual.
7. Criação de um callback para a posição atual, definindo quais valores são enviados na mensagem do publisher.
