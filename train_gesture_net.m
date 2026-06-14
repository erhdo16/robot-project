% ========================================================================
%  train_gesture_net.m — 手势识别网络训练脚本
%
%  流程:
%    1. 准备数据集（自己拍照 或 用内置采集工具）
%    2. 基于 SqueezeNet 迁移学习（轻量，适合机载i5）
%    3. 导出为 .mat 供实时推理脚本调用
%
%  数据集结构要求:
%    gesture_dataset/
%      0/  → 数字0的手势图片（建议每类50-100张）
%      1/
%      2/
%      ...
%      9/
%
%  Quick Start:
%    先运行 collect_gesture_data.m 采集数据
%    再运行本脚本训练
%    最后把 robot_cv_interact.m 里的检测函数替换掉
% ========================================================================

clear; clc; close all;

% ────────────────────────────────────────────────────────────────────────
%  0. 选择基础网络
%     'squeezenet'  — 最轻量，~5MB，i5上单帧<10ms  ← 推荐机载
%     'googlenet'   — 较准确，~27MB，单帧~25ms
%     'mobilenetv2' — 平衡，~14MB，单帧~15ms
% ────────────────────────────────────────────────────────────────────────
BASE_NET     = 'squeezenet';
DATASET_PATH = 'gesture_dataset';
OUTPUT_PATH  = 'gesture_net.mat';
IMG_SIZE     = [227 227];   % squeezenet/googlenet 输入尺寸
NUM_CLASSES  = 10;          % 0-9

% ────────────────────────────────────────────────────────────────────────
%  1. 检查数据集
% ────────────────────────────────────────────────────────────────────────
if ~isfolder(DATASET_PATH)
    error(['找不到数据集文件夹: %s\n' ...
           '请先运行 collect_gesture_data.m 采集数据，\n' ...
           '或手动创建 gesture_dataset/0/ ~ gesture_dataset/9/ 并放入图片。'], ...
           DATASET_PATH);
end

imds = imageDatastore(DATASET_PATH, ...
    'IncludeSubfolders', true, ...
    'LabelSource', 'foldernames');

% 统计各类数量
label_count = countEachLabel(imds);
fprintf('数据集统计:\n');
disp(label_count);

min_count = min(label_count.Count);
if min_count < 20
    warning('部分类别图片不足20张，建议至少50张/类以保证准确率');
end

% 按 80/20 分训练集和验证集，保持各类比例一致
[imds_train, imds_val] = splitEachLabel(imds, 0.8, 'randomized');
fprintf('训练集: %d张，验证集: %d张\n', numel(imds_train.Files), numel(imds_val.Files));

% ────────────────────────────────────────────────────────────────────────
%  2. 数据增强（扩充训练集，提升泛化能力）
%     对手势图片做随机翻转/旋转/缩放，模拟不同拍摄角度
% ────────────────────────────────────────────────────────────────────────
augmenter = imageDataAugmenter( ...
    'RandXReflection',  true, ...          % 左右镜像（注意：会影响单手手势方向）
    'RandRotation',     [-15, 15], ...     % 随机旋转 ±15°
    'RandScale',        [0.85, 1.15], ...  % 随机缩放
    'RandXTranslation', [-20, 20], ...     % 随机平移
    'RandYTranslation', [-20, 20]);

auds_train = augmentedImageDatastore(IMG_SIZE, imds_train, ...
    'DataAugmentation', augmenter, ...
    'ColorPreprocessing', 'gray2rgb');     % 确保3通道

auds_val = augmentedImageDatastore(IMG_SIZE, imds_val, ...
    'ColorPreprocessing', 'gray2rgb');

% ────────────────────────────────────────────────────────────────────────
%  3. 加载预训练网络并修改分类头
% ────────────────────────────────────────────────────────────────────────
fprintf('\n正在加载 %s ...\n', BASE_NET);
net = eval(BASE_NET);   % 调用 squeezenet() / googlenet() / mobilenetv2()

% 找到最后分类层的名称（不同网络不同）
switch BASE_NET
    case 'squeezenet'
        feature_layer   = 'drop9';
        % squeezenet 用卷积层替换，不用全连接
        new_layers = [
            convolution2dLayer(1, NUM_CLASSES, 'Name','new_conv', ...
                'WeightLearnRateFactor',10,'BiasLearnRateFactor',10)
            reluLayer('Name','new_relu')
            globalAveragePooling2dLayer('Name','new_gap')
            softmaxLayer('Name','new_softmax')
            classificationLayer('Name','new_classoutput')
        ];
        lgraph = layerGraph(net);
        lgraph = replaceLayer(lgraph, 'conv10',     new_layers(1));
        lgraph = replaceLayer(lgraph, 'relu_conv10',new_layers(2));
        lgraph = replaceLayer(lgraph, 'pool10',     new_layers(3));
        lgraph = replaceLayer(lgraph, 'prob',       new_layers(4));
        lgraph = replaceLayer(lgraph, 'ClassificationLayer_predictions', new_layers(5));

    case 'googlenet'
        lgraph = layerGraph(net);
        new_fc  = fullyConnectedLayer(NUM_CLASSES, 'Name','new_fc', ...
                      'WeightLearnRateFactor',10,'BiasLearnRateFactor',10);
        new_cls = classificationLayer('Name','new_classoutput');
        lgraph  = replaceLayer(lgraph,'loss3-classifier', new_fc);
        lgraph  = replaceLayer(lgraph,'output',           new_cls);

    case 'mobilenetv2'
        lgraph = layerGraph(net);
        new_fc  = fullyConnectedLayer(NUM_CLASSES, 'Name','new_fc', ...
                      'WeightLearnRateFactor',10,'BiasLearnRateFactor',10);
        new_cls = classificationLayer('Name','new_classoutput');
        lgraph  = replaceLayer(lgraph,'Logits',           new_fc);
        lgraph  = replaceLayer(lgraph,'ClassificationLayer_Logits', new_cls);
end

% ────────────────────────────────────────────────────────────────────────
%  4. 训练选项
% ────────────────────────────────────────────────────────────────────────
opts = trainingOptions('adam', ...
    'MiniBatchSize',        32, ...
    'MaxEpochs',            20, ...
    'InitialLearnRate',     1e-4, ...
    'LearnRateSchedule',    'piecewise', ...
    'LearnRateDropFactor',  0.5, ...
    'LearnRateDropPeriod',  8, ...          % 每8个epoch降一次学习率
    'L2Regularization',     1e-4, ...
    'ValidationData',       auds_val, ...
    'ValidationFrequency',  10, ...
    'ValidationPatience',   5, ...          % 验证集5次不改善则早停
    'Shuffle',              'every-epoch',...
    'Plots',                'training-progress', ...
    'Verbose',              true, ...
    'ExecutionEnvironment', 'auto');        % 自动选CPU/GPU

% ────────────────────────────────────────────────────────────────────────
%  5. 开始训练
% ────────────────────────────────────────────────────────────────────────
fprintf('\n开始训练，请稍候（i5 CPU约需10-20分钟）...\n');
tic;
[trained_net, info] = trainNetwork(auds_train, lgraph, opts);
elapsed = toc;
fprintf('\n训练完成！用时 %.1f 分钟\n', elapsed/60);

% ────────────────────────────────────────────────────────────────────────
%  6. 验证集评估
% ────────────────────────────────────────────────────────────────────────
fprintf('\n正在评估验证集...\n');
pred_labels = classify(trained_net, auds_val, ...
    'ExecutionEnvironment','auto', ...
    'MiniBatchSize', 32);
true_labels = imds_val.Labels;

acc = mean(pred_labels == true_labels);
fprintf('验证集准确率: %.2f%%\n', acc*100);

% 混淆矩阵（直观看哪两个手势容易混淆）
figure('Name','混淆矩阵','Position',[100 100 600 550]);
cm = confusionchart(true_labels, pred_labels, ...
    'Title', sprintf('手势识别混淆矩阵 (准确率 %.1f%%)', acc*100), ...
    'RowSummary','row-normalized', ...
    'ColumnSummary','column-normalized');

% ────────────────────────────────────────────────────────────────────────
%  7. 保存模型
%     保存网络 + 输入尺寸 + 类别标签，供实时推理调用
% ────────────────────────────────────────────────────────────────────────
class_names = string(categories(imds.Labels));
img_size    = IMG_SIZE;
save(OUTPUT_PATH, 'trained_net', 'class_names', 'img_size', '-v7.3');
fprintf('\n模型已保存至: %s\n', OUTPUT_PATH);
fprintf('请将此文件与 robot_cv_interact.m 放在同一目录下运行。\n');
