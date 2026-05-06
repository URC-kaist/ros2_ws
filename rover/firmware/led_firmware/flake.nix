{
  description = "Development shell for the MR2 LED STM32G431 Arduino firmware";

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
              arduino-cli
              probe-rs-tools
              picocom
            ];

            shellHook = ''
              echo "MR2 LED firmware shell"
              echo "  configure Arduino core/libraries with arduino-cli"
              echo "  sketch: led_firmware.ino"
            '';
          };
        });
    };
}
