{
  description = "Development shell for the MR2 battery STM32H523 firmware";

  inputs.nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

  outputs = { self, nixpkgs }:
    let
      supportedSystems = [
        "aarch64-darwin"
        "x86_64-darwin"
        "x86_64-linux"
        "aarch64-linux"
      ];
      forAllSystems = nixpkgs.lib.genAttrs supportedSystems;
    in
    {
      devShells = forAllSystems (system:
        let
          pkgs = import nixpkgs { inherit system; };
        in
        {
          default = pkgs.mkShell {
            packages = with pkgs; [
              cmake
              gcc-arm-embedded
              ninja
              probe-rs-tools
              picocom
            ];

            PROBE_RS_CHIP = "STM32H523CE";

            shellHook = ''
              echo "MR2 battery firmware shell"
              echo "  build: cmake --preset Debug && cmake --build --preset Debug"
              echo "  flash: cmake --build --preset Debug --target flash"
            '';
          };
        });
    };
}
