classdef PriorityQueue < handle
    
    % Priority queue using binary heap. Require elements of queue to be
    % structs with a priority field
    
    properties %(Access = private)
        q = cell(1,20);
        size = 0;
        capacity = 20;
        type = 'min';
        
        % function handle for comparisons for min or max heap
        switchParent
    end
    
    methods
        % Constructor
        function obj = PriorityQueue(capacity,type)
            if nargin < 1
                capacity = 20;
            end
            
            if nargin < 2
                type = 'min';
            end
            
            obj.capacity = capacity;
            obj.type = type;
            obj.size = 0;
            obj.q = cell(1,capacity);
            
            if strcmp(type,'min')
                obj.switchParent = ...
                    @(parent,child) parent.priority > child.priority;
            elseif strcmp(type,'max')
                obj.switchParent = ...
                    @(parent,child) parent.priority < child.priority;
            else
                error('Type must be ''min'' or ''max''')
            end
            
        end
        
        % getters
        function s = getSize(obj)
            s = obj.size;
        end
        function c = getCapacity(obj)
            c = obj.capacity;
        end
        function t = getType(obj)
            t = obj.type;
        end
        
        % To priority array. Returns queue priority values in array
        function p = getPriorityArray(obj)
            p = nan(1,obj.size);
            for i = 1:obj.size
                p(i) = obj.q{i}.priority;
            end
        end
        % To cell array. Returns queue
        function a = toCellArray(obj)
            a = obj.q(1:obj.size);
        end
        
        % Enqueue
        function added = add(obj,newElem)
            added = true;
            if ~isstruct(newElem)
                error('New element must be a struct with a ''priority'' field.')
            elseif ~isfield(newElem,'priority')
                error('New element must have a ''priority'' field.')
            end
            
            % If capacity reached, increase by 50%
            if obj.size == obj.capacity
                newSpace = ceil(obj.capacity*0.5);
                obj.capacity = obj.capacity+newSpace;
                obj.q = horzcat(obj.q, cell(1,newSpace));
            end
            
            % Add element to binary heap and bubble up
            obj.size = obj.size+1;
            index = obj.size;
            obj.q{index} = newElem;
            parentIndex = floor(index/2);
            while parentIndex > 0 && obj.switchParent(obj.q{parentIndex},obj.q{index}) 
                tempElem = obj.q{index};
                obj.q{index} = obj.q{parentIndex};
                obj.q{parentIndex} = tempElem;

                index = parentIndex;
                parentIndex = floor(index/2);
            end
        end
        
        % Dequeue
        function elem = dequeue(obj)
            % Return the first element. Will be empty if size is 0
            elem = obj.q{1};
            
            % If there are zero elements, return
            if obj.size < 1
                return
            end
            
            % Move last element to front of queue and trickle down
            obj.q{1} = obj.q{obj.size};
            obj.q{obj.size} = [];
            obj.size = obj.size - 1;
            parentIndex = 1;
            leftChildIndex = parentIndex*2;
            rightChildIndex = leftChildIndex + 1;
            while leftChildIndex <= obj.size
                tempElem = obj.q{parentIndex};
                
                switchChildIndex = leftChildIndex;
                if rightChildIndex <= obj.size &&...
                   obj.switchParent(obj.q{leftChildIndex},obj.q{rightChildIndex}) 
                    
                   switchChildIndex = rightChildIndex;
                end
                
                if obj.switchParent(obj.q{parentIndex},obj.q{switchChildIndex})
                    obj.q{parentIndex} = obj.q{switchChildIndex};
                    obj.q{switchChildIndex} = tempElem;
                    parentIndex = switchChildIndex;
                else
                    break
                end
                
                leftChildIndex = parentIndex*2;
                rightChildIndex = leftChildIndex + 1;
            end
        end
    end
end