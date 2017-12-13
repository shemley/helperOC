classdef NeuralNode < handle
    
    % Neural network node
    
    properties %(Access = private)
        type
        value
        grad
        valFunc
        gradFunc
        regularizationRate
        children
    end
    
    methods
        % Constructor
        function obj = NeuralNode(type,initVal,regularizationRate)          
            obj.type = type;
            obj.value = initVal;
            
            if nargin < 3
                regularizationRate = 0;
            end
            
            obj.regularizationRate = regularizationRate;
            obj.children = {};
            
            if strcmp(type,'add')
                obj.valFunc = @(x,y) sum(x);
                obj.gradFunc = @(varargin) 1;
            elseif strcmp(type,'sub')
                obj.valFunc = @(x,y) x - y;
                obj.gradFunc = @(x,y,which) (which == 1) - (which == 2);
            elseif strcmp(type,'dot')
                obj.valFunc = @(x,y) dot(x,y);
                obj.gradFunc = @(x,y,which) (which == 1)*y + (which == 2)*x;
            elseif strcmp(type,'square')
                obj.valFunc = @(x,y) 0.5*x.^2;
                obj.gradFunc = @(x,y,which) 2*x;
            elseif strcmp(type,'sigmoid')
                obj.valFunc = @(x,y) 1./(1+exp(-x));
                obj.gradFunc = @(x,y,which) obj.valFunc(x)*(1-obj.valFunc(x));
            elseif strcmp(type,'weight')
                obj.valFunc = @(x,y) x;
                obj.gradFunc = @(x,y,which) x;
            elseif strcmp(type,'feature')
                obj.valFunc = @(x,y) x;
                obj.gradFunc = @(x,y,which) x;
            elseif strcmp(type,'target')
                obj.valFunc = @(x,y) x;
                obj.gradFunc = @(x,y,which) x;
            else
                error('Invalid node type')
            end
            
        end
        
        % getters
        function v = getValue(obj)
            v = obj.value;
        end
        function g = getGrad(obj)
            g = obj.grad;
        end
        function t = getType(obj)
            t = obj.type;
        end
        function r = getRegRate(obj)
            r = obj.regularizationRate;
        end
        function c = getChildren(obj)
            c = obj.children;
        end
                        
        % add child
        function newChild = addChild(obj,type,initVal)
            newChild = NeuralNode(type,initVal,obj.regularizationRate);
            obj.children = horzcat(obj.children,{newChild});
        end
        
        % update values
        function v = calcValue(obj,features,target)
            if strcmp(obj.type,'weight')
                v = obj.value;
            elseif strcmp(obj.type,'feature')
                v = features;
            elseif strcmp(obj.type,'target')
                v = target;                
            elseif strcmp(obj.type,'add')
                valInput = zeros(size(obj.children));
                for i = 1:length(obj.children)
                    valInput(i) = obj.children{i}.calcValue(features,target);
                end
                v = obj.valFunc(valInput);
            else
                if length(obj.children) < 2
                    v = obj.valFunc(obj.children{1}.calcValue(features,target));
                else
                    v = obj.valFunc(...
                        obj.children{1}.calcValue(features,target),...
                        obj.children{2}.calcValue(features,target));
                end
            end
        end
        
        % update gradients
        function calcGrad(obj,pastGrad,eta)
            obj.grad = pastGrad;
            
            if strcmp(obj.type,'weight')
                % update weights by gradient descent
                regTerm = obj.regularizationRate*obj.value;
                obj.value = obj.value - eta*(obj.grad + regTerm);
            elseif strcmp(obj.type,'feature')
                return
            else
                numChildren = length(obj.children);
                
                for i = 1:numChildren
                    if numChildren == 1 || numChildren > 2
                        newGrad = obj.gradFunc(obj.children{1}.getValue());
                    else                    
                        newGrad = obj.gradFunc(obj.children{1}.getValue(),...
                        obj.children{2}.getValue(),i);  
                    end
                    obj.children{i}.calcGrad(pastGrad.*newGrad, eta);
                end
            end
        end
    end
end