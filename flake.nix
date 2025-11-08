{
  description = "C environment";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-25.05";
  };

  outputs = { self, nixpkgs, ... }:

      let
        system = "x86_64-linux";
        pkgs = import nixpkgs {
          inherit system;
          config = { allowUnfree = true; };
        };

		
		
        extensions = with pkgs.vscode-extensions; [
	  ms-vscode.cpptools
	  twxs.cmake
          pkief.material-icon-theme
        ];

          
        
        vscode-with-extensions =
          pkgs.vscode-with-extensions.override { vscodeExtensions = extensions; };
        
      in
      with pkgs;
      {
        devShell.x86_64-linux = mkShell {
          buildInputs = [
            bashInteractive # needed for vscode integrated terminal to work properly
            gcc
            gdb
            gnumake
            libGL
            libGLU
            mesa
            glfw
            glew
            glm
            xorg.libXxf86vm
            vscode-with-extensions
            xorg.libX11
            xorg.libXext
            xorg.libXinerama
            xorg.libXrandr
            xorg.libXcursor
            xorg.libXxf86vm
            glxinfo
	    autoPatchelfHook
          ];

        shellHook =
        ''
        export FLAKE_ROOT=$(git rev-parse --show-toplevel)
        export LD_LIBRARY_PATH=${pkgs.libGL}/lib:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=${pkgs.xorg.libX11}/lib:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=${pkgs.xorg.libXext}/lib:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=${pkgs.xorg.libXinerama}/lib:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=${pkgs.xorg.libXrandr}/lib:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=${pkgs.xorg.libXcursor}/lib:$LD_LIBRARY_PATH
        export LD_LIBRARY_PATH=${pkgs.xorg.libXxf86vm}/lib:$LD_LIBRARY_PATH
        '';
        };
      };
}
