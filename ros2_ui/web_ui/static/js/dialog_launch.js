class DialogLaunch {
    dialog = $("#launch_dialog")
    open() {
        this.dialog.get(0).showModal()
    }
    close() {
        this.dialog.get(0).close()
    }
}

dialog_launch = new DialogLaunch()

