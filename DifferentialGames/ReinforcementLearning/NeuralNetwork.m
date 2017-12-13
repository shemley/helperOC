classdef NeuralNetwork < handle
    
    % Neural network
    
    properties %(Access = private)
        hiddenLayers = 1;
        nodesPerLayer = 4;
        numFeatures = 5;
        regularizationRate
        root
        valueNode
    end
    
    methods
        % Constructor
        function obj = NeuralNetwork(numFeatures,hiddenLayers,nodesPerLayer,regularizationRate)
            if nargin < 1
                numFeatures = 5;
            end
            
            if nargin < 2
                hiddenLayers = 1;
            end
            
            if nargin < 3
                nodesPerLayer = 4;
            end
            
            if nargin < 4
                regularizationRate = 0;
            end
                        
            obj.numFeatures = numFeatures;
            obj.hiddenLayers = hiddenLayers;
            obj.nodesPerLayer = nodesPerLayer;
            obj.regularizationRate = regularizationRate;
            
            % build root network
            obj.root = NeuralNode('square',rand(1),obj.regularizationRate);
            subNode = obj.root.addChild('sub',rand(1));
            obj.valueNode = subNode.addChild('add',rand(1));
            subNode.addChild('target',rand(1));
            
            % build value network
            leaves = cell(1,obj.nodesPerLayer);
            for i = 1:obj.nodesPerLayer
                leaves{i} = obj.valueNode.addChild('dot',rand(1));
            end            
            % Add hidden layers
            for i = 1:obj.hiddenLayers                
                for j = 1:obj.nodesPerLayer
                    leaves{j}.addChild('weight',rand(1));
                    leaves{j} = leaves{j}.addChild('sigmoid',rand(1));
                end
            end
            % add direct layer
            for i = 1:obj.nodesPerLayer
                leaves{i} = leaves{i}.addChild('dot',rand(1));
                leaves{i}.addChild('weight',rand(numFeatures,1));
                leaves{i}.addChild('feature',rand(numFeatures,1));
            end
        end
        
        % getters
        function nf = getNumFeatures(obj)
            nf = obj.numFeatures;
        end
        function npl = getNodesPerLayer(obj)
            npl = obj.nodesPerLayer;
        end
        function hl = getNumHiddenLayers(obj)
            hl = obj.hiddenLayers;
        end
        function r = getRegularizationRate(obj)
            r = obj.regularizationRate;
        end
        function V = getValue(obj)
            V = obj.valueNode.getValue();
        end
        
        % calc value
        function V = calculateValue(obj,featureVec)
            V = obj.valueNode.calcValue(featureVec,0);
        end
        
        % backprop
        function backprop(obj,featureVec,target,eta)
            obj.root.calcValue(featureVec,target);
            obj.root.calcGrad(1,eta);
        end

    end
end