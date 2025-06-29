How to Incorporate Legacy Robots in Webots 2025a

To adapt these robots for use in Webots 2025a:

    Move PROTOs out of protos/ignore/ into protos/robots/

    Edit the .proto file(s):

        Remove or replace expressions like %{fields...}

        Replace template-driven logic with static Webots field definitions

    Ensure referenced meshes and textures exist

        Relative paths like ../meshes/part.dae must point to real files

    Test by importing into a .wbt world

        Use IMPORTABLE EXTERNPROTO and manually insert into the Scene Tree
