# 11 — Dataset e Treinamento

Este documento descreve todo o fluxo de preparação de dados e treinamento do
modelo YOLO v8 para detecção do landing pad, incluindo o diagnóstico do
problema inicial, a montagem dos conjuntos de treino e validação, os comandos
de treino e os resultados de avaliação.

---

## 1. Classes do dataset

O modelo detecta **3 classes** do landing pad:

| ID | Nome | Descrição |
|----|------|-----------|
| `0` | `base` | Estrutura principal do pad (quadrado/círculo) |
| `1` | `h` | Marcador em forma de H no centro do pad |
| `2` | `borda` | Borda/contorno do pad |

---

## 2. Problema inicial: dataset grande sem classes 1 e 2

### 2.1 Situação

O dataset de treino original estava localizado em:

```
Imagens: /home/lmnr31/datasets/landing_pad/dataset/images/train
Labels:  /home/lmnr31/datasets/landing_pad/dataset/labels/train
```

### 2.2 Diagnóstico

Ao inspecionar as labels, verificou-se que:

- **9 451** arquivos de label no total
- **8 993** labels vazias (apenas background, sem anotações)
- Apenas a **classe 0** (`base`) estava presente: `{0: 897}`
- As classes `1` (`h`) e `2` (`borda`) **não existiam** no conjunto de treino

Diagnóstico confirmado pelo comando:

```bash
# Busca por linhas que começam com "1 " ou "2 " (classes 1 e 2 no formato YOLO)
grep -R "^[12] " -n /home/lmnr31/datasets/landing_pad/dataset/labels/train | head
```

**Resultado: nenhuma saída** — confirmando ausência total das classes 1 e 2.

### 2.3 Consequência

O modelo treinado com esse dataset não aprendia a detectar o marcador H nem a
borda do pad. Ao validar em imagens com anotações completas, a métrica
`AP` (Average Precision) para a classe `h` era **0**.

---

## 3. Montagem do conjunto de validação (val800) a partir do CVAT

### 3.1 Fontes de dados CVAT

```
Imagens CVAT: /home/lmnr31/datasets/landing_pad/cvat_batches/train_1
Labels CVAT:  /home/lmnr31/datasets/landing_pad/cvat_zip_raw/obj_train_data
```

### 3.2 Script de montagem

Foi escrito um script Python que:

1. Indexa as imagens CVAT por **stem** (nome sem extensão)
2. Copia pares `(imagem, label)` para o dataset de validação, garantindo
   que cada label tenha a imagem correspondente

### 3.3 Saída gerada

```
/home/lmnr31/datasets/landing_pad/val_from_cvat/
├── images/val/   ← 800 imagens
└── labels/val/   ← 800 labels
```

### 3.4 Estatísticas do val800

| Métrica | Valor |
|---------|-------|
| Total de labels | 800 |
| Labels não-vazias | 511 |
| Labels de background | 289 |

---

## 4. Tentativa de treino misto (e por que falhou)

Tentou-se combinar:
- **Treino**: dataset grande (9 451 imagens, só classe 0)
- **Val**: `val_from_cvat` (3 classes)

O treino executava normalmente, mas a validação mostrava:
- `h` = AP 0 (classe nunca vista no treino)
- `borda` instável / desempenho inconsistente

**Conclusão**: impossível aprender classes que não existem no conjunto de
treino, independentemente do conjunto de validação.

---

## 5. Solução: dataset CVAT completo no formato Ultralytics

### 5.1 Criação do dataset

Foi criado um dataset Ultralytics completo **exclusivamente a partir das
anotações CVAT**, com split 80 % treino / 20 % validação:

```
/home/lmnr31/datasets/landing_pad/cvat_ultra/
├── images/
│   ├── train/   ← 80 % das imagens CVAT
│   └── val/     ← 20 % das imagens CVAT
└── labels/
    ├── train/   ← labels YOLO correspondentes
    └── val/     ← labels YOLO correspondentes
```

### 5.2 Arquivo YAML do dataset

```yaml
# /home/lmnr31/datasets/landing_pad/cvat_ultra.yaml
path: /home/lmnr31/datasets/landing_pad/cvat_ultra
train: images/train
val:   images/val

names:
  0: base
  1: h
  2: borda
```

---

## 6. Treinamento

### 6.1 Ativação do ambiente

```bash
source /home/lmnr31/venvs/yolo/bin/activate
```

### 6.2 Comando de treino

```bash
yolo detect train \
  data=/home/lmnr31/datasets/landing_pad/cvat_ultra.yaml \
  model=yolov8n.pt \
  epochs=100 \
  patience=15 \
  imgsz=640 \
  batch=16 \
  name=train7
```

| Argumento | Valor | Significado |
|-----------|-------|-------------|
| `data` | `cvat_ultra.yaml` | Caminho para o YAML do dataset |
| `model` | `yolov8n.pt` | Modelo base (nano; menor e mais rápido) |
| `epochs` | `100` | Número máximo de épocas |
| `patience` | `15` | EarlyStopping: para se `mAP50` não melhorar em 15 épocas consecutivas |
| `imgsz` | `640` | Tamanho de entrada da imagem (pixels) |
| `batch` | `16` | Tamanho do batch |
| `name` | `train7` | Nome do run (salvo em `~/runs/detect/train7/`) |

### 6.3 EarlyStopping

O treino parou automaticamente por `EarlyStopping(patience=15)` antes de
atingir as 100 épocas, salvando o melhor checkpoint como:

```
/home/lmnr31/runs/detect/train7/weights/best.pt
```

---

## 7. Avaliação no val800

### 7.1 YAML de avaliação

```yaml
# /home/lmnr31/datasets/landing_pad/eval_val800.yaml
path: /home/lmnr31/datasets/landing_pad/val_from_cvat
val: images/val

names:
  0: base
  1: h
  2: borda
```

### 7.2 Comando de validação

```bash
yolo detect val \
  model=/home/lmnr31/runs/detect/train7/weights/best.pt \
  data=/home/lmnr31/datasets/landing_pad/eval_val800.yaml \
  imgsz=640
```

### 7.3 Resultados obtidos

| Classe | P | R | mAP50 | mAP50-95 |
|--------|---|---|-------|----------|
| **all** | 0.969 | 0.976 | 0.988 | 0.782 |
| base | — | — | — | 0.827 |
| h | — | — | — | 0.700 |
| borda | — | — | — | 0.817 |

**Interpretação:**

- `P = 0.969`: 96,9 % das detecções são verdadeiros positivos
- `R = 0.976`: 97,6 % dos objetos reais foram detectados
- `mAP50 = 0.988`: precisão média a IoU ≥ 0.50 de 98,8 %
- `mAP50-95 = 0.782`: precisão média integrada de IoU 0.50 a 0.95
- `h` com `mAP50-95 = 0.700`: classe mais difícil (menor objeto no pad),
  mas voltou a ser detectada após o dataset CVAT completo

---

## 8. Localização dos artefatos finais

| Artefato | Caminho |
|----------|---------|
| Dataset de treino/val | `/home/lmnr31/datasets/landing_pad/cvat_ultra/` |
| YAML de treino | `/home/lmnr31/datasets/landing_pad/cvat_ultra.yaml` |
| Dataset de avaliação real | `/home/lmnr31/datasets/landing_pad/val_from_cvat/` |
| YAML de avaliação | `/home/lmnr31/datasets/landing_pad/eval_val800.yaml` |
| Pesos finais | `/home/lmnr31/runs/detect/train7/weights/best.pt` |
