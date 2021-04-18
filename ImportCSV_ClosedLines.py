import adsk.core, adsk.fusion, traceback
import io

def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui  = app.userInterface
        product = app.activeProduct
        design = adsk.fusion.Design.cast(product)
        title = 'Import points from csv to make lines'
        if not design:
            ui.messageBox('No active Fusion design', title)
            return

        dlg = ui.createFileDialog()
        dlg.title = 'Open CSV File'
        dlg.filter = 'Comma Separated Values (*.csv);;All Files (*.*)'
        if dlg.showOpen() != adsk.core.DialogResults.DialogOK :
            return

        filename = dlg.filename
        terminal_flg = True
        with io.open(filename, 'r', encoding='utf-8-sig') as f:
            root = design.rootComponent
            sketch = root.sketches.add(root.xYConstructionPlane)
            line = f.readline()
            data = []
            while line:
                pntStrArr = line.split(',')
                for pntStr in pntStrArr:
                    try:
                        data.append(float(pntStr))
                    except:
                        break

                if len(data) >= 3 :
                    sketchLines = sketch.sketchCurves.sketchLines
                    endPoint = adsk.core.Point3D.create(data[0], data[1], data[2])
                    if terminal_flg == True:
                        terminal_flg = False
                        terminalPoint = adsk.core.Point3D.create(data[0], data[1], data[2])
                    else :
                        sketchLines.addByTwoPoints(startPoint, endPoint)
                    startPoint = endPoint
                line = f.readline()
                data.clear()
            sketchLines.addByTwoPoints(endPoint, terminalPoint)

    except:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
