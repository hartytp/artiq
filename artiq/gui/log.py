import logging
import time
from functools import partial

from PyQt6 import QtCore, QtGui, QtWidgets

from sipyco.logs import SourceFilter
from artiq.gui.tools import (LayoutWidget, log_level_to_name,
                             QDockWidgetCloseDetect)


class _ModelItem:
    def __init__(self, parent, row):
        self.parent = parent
        self.row = row
        self.children_by_row = []


class _LogFilterProxyModel(QtCore.QSortFilterProxyModel):
    def __init__(self):
        super().__init__()
        self.setFilterCaseSensitivity(QtCore.Qt.CaseSensitivity.CaseInsensitive)
        self.setRecursiveFilteringEnabled(True)
        self.filter_level = 0

    def filterAcceptsRow(self, source_row, source_parent):
        source = self.sourceModel()
        index0 = source.index(source_row, 0, source_parent)
        index1 = source.index(source_row, 1, source_parent)
        level = source.data(index0, QtCore.Qt.ItemDataRole.UserRole)

        if level >= self.filter_level:
            regex = self.filterRegularExpression()
            index0_text = source.data(index0, QtCore.Qt.ItemDataRole.DisplayRole)
            msg_text = source.data(index1, QtCore.Qt.ItemDataRole.DisplayRole)
            return (regex.match(index0_text).hasMatch() or regex.match(msg_text).hasMatch())
        else:
            return False

    def apply_filter_level(self, filter_level):
        self.filter_level = getattr(logging, filter_level)
        self.invalidateFilter()


class _Model(QtCore.QAbstractItemModel):
    def __init__(self, palette):
        QtCore.QAbstractTableModel.__init__(self)

        self.headers = ["Source", "Message"]
        self.children_by_row = []

        self.entries = []
        self.pending_entries = []
        self.depth = 1000
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.timer_tick)
        timer.start(100)

        self.fixed_font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.SystemFont.FixedFont)

        self.default_bg = palette.base()
        self.default_fg = palette.text()
        self.debug_fg = palette.placeholderText()
        is_dark_mode = self.default_bg.color().lightness() < self.default_fg.color().lightness()
        if is_dark_mode:
            self.warning_bg = QtGui.QBrush(QtGui.QColor(90, 74, 0))
            self.error_bg = QtGui.QBrush(QtGui.QColor(98, 24, 24))
        else:
            self.warning_bg = QtGui.QBrush(QtGui.QColor(255, 255, 180))
            self.error_bg = QtGui.QBrush(QtGui.QColor(255, 150, 150))

    def headerData(self, col, orientation, role):
        if (orientation == QtCore.Qt.Orientation.Horizontal
                and role == QtCore.Qt.ItemDataRole.DisplayRole):
            return self.headers[col]
        return None

    def rowCount(self, parent):
        if parent.isValid():
            item = parent.internalPointer()
            return len(item.children_by_row)
        else:
            return len(self.entries)

    def columnCount(self, parent):
        return len(self.headers)

    def append(self, v):
        severity, source, timestamp, message = v
        self.pending_entries.append((severity, source, timestamp,
                                     message.splitlines()))

    def clear(self):
        self.beginRemoveRows(QtCore.QModelIndex(), 0, len(self.entries)-1)
        self.entries.clear()
        self.children_by_row.clear()
        self.endRemoveRows()

    def timer_tick(self):
        if not self.pending_entries:
            return
        nrows = len(self.entries)
        records = self.pending_entries
        self.pending_entries = []

        self.beginInsertRows(QtCore.QModelIndex(), nrows, nrows+len(records)-1)
        self.entries.extend(records)
        for rec in records:
            item = _ModelItem(self, len(self.children_by_row))
            self.children_by_row.append(item)
            for i in range(len(rec[3])-1):
                item.children_by_row.append(_ModelItem(item, i))
        self.endInsertRows()

        if len(self.entries) > self.depth:
            start = len(self.entries) - self.depth
            self.beginRemoveRows(QtCore.QModelIndex(), 0, start-1)
            self.entries = self.entries[start:]
            self.children_by_row = self.children_by_row[start:]
            for child in self.children_by_row:
                child.row -= start
            self.endRemoveRows()

    def index(self, row, column, parent):
        if parent.isValid():
            parent_item = parent.internalPointer()
            return self.createIndex(row, column,
                                    parent_item.children_by_row[row])
        else:
            return self.createIndex(row, column, self.children_by_row[row])

    def parent(self, index):
        if index.isValid():
            parent = index.internalPointer().parent
            if parent is self:
                return QtCore.QModelIndex()
            else:
                return self.createIndex(parent.row, 0, parent)
        else:
            return QtCore.QModelIndex()

    def full_entry(self, index):
        if not index.isValid():
            return
        item = index.internalPointer()
        if item.parent is self:
            msgnum = item.row
        else:
            msgnum = item.parent.row
        return self.entries[msgnum][3]

    def data(self, index, role):
        if not index.isValid():
            return

        item = index.internalPointer()
        if item.parent is self:
            msgnum = item.row
        else:
            msgnum = item.parent.row

        if role == QtCore.Qt.ItemDataRole.FontRole and index.column() == 1:
            return self.fixed_font
        elif role == QtCore.Qt.ItemDataRole.BackgroundRole:
            level = self.entries[msgnum][0]
            if level >= logging.ERROR:
                return self.error_bg
            elif level >= logging.WARNING:
                return self.warning_bg
            else:
                return self.default_bg
        elif role == QtCore.Qt.ItemDataRole.ForegroundRole:
            level = self.entries[msgnum][0]
            if level <= logging.DEBUG:
                return self.debug_fg
            else:
                return self.default_fg
        elif role == QtCore.Qt.ItemDataRole.DisplayRole:
            v = self.entries[msgnum]
            column = index.column()
            if item.parent is self:
                if column == 0:
                    return v[1]
                else:
                    return v[3][0]
            else:
                if column == 0:
                    return ""
                else:
                    return v[3][item.row+1]
        elif role == QtCore.Qt.ItemDataRole.ToolTipRole:
            v = self.entries[msgnum]
            if item.parent is self:
                lineno = 0
            else:
                lineno = item.row + 1
            return (log_level_to_name(v[0]) + ", " +
                time.strftime("%m/%d %H:%M:%S", time.localtime(v[2])) +
                "\n" + v[3][lineno])
        elif role == QtCore.Qt.ItemDataRole.UserRole:
            return self.entries[msgnum][0]


class LogDock(QDockWidgetCloseDetect):
    def __init__(self, manager, name):
        QDockWidgetCloseDetect.__init__(self, "Log")
        self.setObjectName(name)

        grid = LayoutWidget()
        self.setWidget(grid)

        grid.addWidget(QtWidgets.QLabel("Minimum level: "), 0, 0)
        self.filter_level = QtWidgets.QComboBox()
        self.filter_level.addItems(["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"])
        self.filter_level.setToolTip("Filter entries at or above this level")
        grid.addWidget(self.filter_level, 0, 1)
        self.filter_freetext = QtWidgets.QLineEdit()
        self.filter_freetext.setPlaceholderText("freetext filter...")
        self.filter_freetext.setToolTip("Filter entries containing this text")
        grid.addWidget(self.filter_freetext, 0, 2)

        scrollbottom = QtWidgets.QToolButton()
        scrollbottom.setToolTip("Scroll to bottom")
        scrollbottom.setIcon(QtWidgets.QApplication.style().standardIcon(
            QtWidgets.QStyle.StandardPixmap.SP_ArrowDown))
        grid.addWidget(scrollbottom, 0, 3)
        scrollbottom.clicked.connect(self.scroll_to_bottom)

        clear = QtWidgets.QToolButton()
        clear.setIcon(QtWidgets.QApplication.style().standardIcon(
            QtWidgets.QStyle.StandardPixmap.SP_DialogResetButton))
        grid.addWidget(clear, 0, 4)
        clear.clicked.connect(lambda: self.model.clear())

        if manager:
            newdock = QtWidgets.QToolButton()
            newdock.setToolTip("Create new log dock")
            newdock.setIcon(QtWidgets.QApplication.style().standardIcon(
                QtWidgets.QStyle.StandardPixmap.SP_FileDialogNewFolder))
            # note the lambda, the default parameter is overriden otherwise
            newdock.clicked.connect(lambda: manager.create_new_dock())
            grid.addWidget(newdock, 0, 5)
        grid.layout.setColumnStretch(2, 1)

        self.log = QtWidgets.QTreeView()
        self.log.setHorizontalScrollMode(
            QtWidgets.QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.log.setVerticalScrollMode(
            QtWidgets.QAbstractItemView.ScrollMode.ScrollPerPixel)
        self.log.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        grid.addWidget(self.log, 1, 0, colspan=6 if manager else 5)
        self.scroll_at_bottom = False
        self.scroll_value = 0

        self.log.setContextMenuPolicy(QtCore.Qt.ContextMenuPolicy.ActionsContextMenu)
        copy_action = QtGui.QAction("Copy entry to clipboard", self.log)
        copy_action.triggered.connect(self.copy_to_clipboard)
        self.log.addAction(copy_action)
        clear_action = QtGui.QAction("Clear", self.log)
        clear_action.triggered.connect(lambda: self.model.clear())
        self.log.addAction(clear_action)

        # If Qt worked correctly, this would be nice to have. Alas, resizeSections
        # is broken when the horizontal scrollbar is enabled.
        # sizeheader_action = QtGui.QAction("Resize header", self.log)
        # sizeheader_action.triggered.connect(
        #     lambda: self.log.header().resizeSections(QtWidgets.QHeaderView.ResizeMode.ResizeToContents))
        # self.log.addAction(sizeheader_action)

        cw = QtGui.QFontMetrics(self.font()).averageCharWidth()
        self.log.header().resizeSection(0, 26*cw)

        self.model = _Model(self.palette())
        self.proxy_model = _LogFilterProxyModel()
        self.proxy_model.setSourceModel(self.model)
        self.log.setModel(self.proxy_model)

        self.model.rowsAboutToBeInserted.connect(self.rows_inserted_before)
        self.model.rowsInserted.connect(self.rows_inserted_after)

        self.filter_freetext.returnPressed.connect(self.apply_text_filter)
        self.filter_level.currentIndexChanged.connect(self.apply_level_filter)

    def apply_text_filter(self):
        self.proxy_model.setFilterRegularExpression(self.filter_freetext.text())

    def apply_level_filter(self):
        self.proxy_model.apply_filter_level(self.filter_level.currentText())

    def scroll_to_bottom(self):
        self.log.scrollToBottom()

    def rows_inserted_before(self):
        scrollbar = self.log.verticalScrollBar()
        self.scroll_value = scrollbar.value()
        self.scroll_at_bottom = self.scroll_value == scrollbar.maximum()

    def rows_inserted_after(self):
        if self.scroll_at_bottom:
            self.log.scrollToBottom()

    def copy_to_clipboard(self):
        idx = self.log.selectedIndexes()
        if idx:
            source_idx = self.proxy_model.mapToSource(idx[0])
            entry = "\n".join(self.model.full_entry(source_idx))
            QtWidgets.QApplication.clipboard().setText(entry)

    def save_state(self):
        return {
            "min_level_idx": self.filter_level.currentIndex(),
            "freetext_filter": self.filter_freetext.text(),
            "header": bytes(self.log.header().saveState())
        }

    def restore_state(self, state):
        try:
            idx = state["min_level_idx"]
        except KeyError:
            pass
        else:
            self.filter_level.setCurrentIndex(idx)

        try:
            freetext = state["freetext_filter"]
        except KeyError:
            pass
        else:
            self.filter_freetext.setText(freetext)

        try:
            header = state["header"]
        except KeyError:
            pass
        else:
            self.log.header().restoreState(QtCore.QByteArray(header))


class LogDockManager:
    def __init__(self, main_window):
        self.main_window = main_window
        self.docks = dict()

    def append_message(self, msg):
        for dock in self.docks.values():
            dock.model.append(msg)

    def create_new_dock(self, add_to_area=True):
        n = 0
        name = "log0"
        while name in self.docks:
            n += 1
            name = "log" + str(n)

        dock = LogDock(self, name)
        self.docks[name] = dock
        if add_to_area:
            self.main_window.addDockWidget(QtCore.Qt.DockWidgetArea.RightDockWidgetArea, dock)
            dock.setFloating(True)
        dock.sigClosed.connect(partial(self.on_dock_closed, name))
        self.update_closable()
        return dock

    def on_dock_closed(self, name):
        dock = self.docks[name]
        dock.deleteLater()
        del self.docks[name]
        self.update_closable()

    def update_closable(self):
        flags = (QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetMovable |
                 QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetFloatable)
        if len(self.docks) > 1:
            flags |= QtWidgets.QDockWidget.DockWidgetFeature.DockWidgetClosable
        for dock in self.docks.values():
            dock.setFeatures(flags)

    def save_state(self):
        return {name: dock.save_state() for name, dock in self.docks.items()}

    def restore_state(self, state):
        if self.docks:
            raise NotImplementedError
        for name, dock_state in state.items():
            dock = LogDock(self, name)
            self.docks[name] = dock
            dock.restore_state(dock_state)
            self.main_window.addDockWidget(QtCore.Qt.DockWidgetArea.RightDockWidgetArea, dock)
            dock.sigClosed.connect(partial(self.on_dock_closed, name))
        self.update_closable()

    def first_log_dock(self):
        if self.docks:
            return None
        dock = self.create_new_dock(False)
        return dock


class LogWidgetHandler(logging.Handler):
    def __init__(self, *args, **kwargs):
        logging.Handler.__init__(self, *args, **kwargs)
        self.callback = None
        self.setFormatter(logging.Formatter("%(name)s:%(message)s"))

    def emit(self, record):
        if self.callback is not None:
            message = self.format(record)
            self.callback((record.levelno, record.source,
                           record.created, message))


def init_log(args, local_source):
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.NOTSET)  # we use our custom filter only
    flt = SourceFilter(logging.INFO + args.quiet*10 - args.verbose*10,
                       local_source)
    handlers = []
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(logging.Formatter(
        "%(levelname)s:%(source)s:%(name)s:%(message)s"))
    handlers.append(console_handler)

    widget_handler = LogWidgetHandler()
    handlers.append(widget_handler)

    for handler in handlers:
        handler.addFilter(flt)
        root_logger.addHandler(handler)

    return widget_handler
