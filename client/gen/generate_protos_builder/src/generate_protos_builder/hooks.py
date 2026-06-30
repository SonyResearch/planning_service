from hatchling.plugin import hookimpl

from .plugin import GenerateProtos


@hookimpl
def hatch_register_build_hook():
    return GenerateProtos
