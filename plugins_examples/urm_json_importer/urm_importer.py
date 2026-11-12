"""Sample importer plugin: reads a URM JSON file and returns the RobotModel."""


def register(host):
    def import_file(path):
        import core_bridge as cb
        text = host.read_text(path)          # needs file_read; scope: the file the user picked
        return cb.urm_from_json(text)

    host.register_importer("URM JSON", "URM JSON (*.json)", import_file)
