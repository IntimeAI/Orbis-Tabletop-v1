import os

import omni.ext
from isaacsim.examples.browser import get_instance as get_browser_instance
from isaacsim.examples.interactive.base_sample import BaseSampleUITemplate
from isaacsim.examples.interactive.table_examples import TablePickup


class TablePickupExtension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        self.example_name = "Table Pickup"
        self.category = "Table Examples"

        ui_kwargs = {
            "ext_id": ext_id,
            "file_path": os.path.abspath(__file__),
            "title": "Table Pickup Example",
            "overview": "This Example provides a simple demonstration of using table data.",
            "sample": TablePickup(),
        }

        ui_handle = BaseSampleUITemplate(**ui_kwargs)

        get_browser_instance().register_example(
            name=self.example_name,
            execute_entrypoint=ui_handle.build_window,
            ui_hook=ui_handle.build_ui,
            category=self.category,
        )

        return

    def on_shutdown(self):
        get_browser_instance().deregister_example(
            name=self.example_name, category=self.category
        )

        return
