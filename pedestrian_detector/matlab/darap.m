%CLASS_INTERFACE Example MATLAB class wrapper to an underlying C++ class
classdef darap < handle
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
    end
    methods
        %% Constructor - Create a new C++ class instance 
        function this = darap(varargin)
            this.objectHandle = darap_interface_mex('new', varargin{:});
        end
        
        %% Destructor - Destroy the C++ class instance
        function delete(this)
            darap_interface_mex('delete', this.objectHandle);
        end
        
        %% Detect persons
        function varargout = get_probability_maps(this, varargin)
            [varargout{1:nargout}] = darap_interface_mex('get_probability_maps', this.objectHandle, varargin{:});
        end


        %% Detect persons
        function varargout = compute_probabilities(this, varargin)
            [varargout{1:nargout}] = darap_interface_mex('compute_probabilities', this.objectHandle, varargin{:});
        end

    end
end