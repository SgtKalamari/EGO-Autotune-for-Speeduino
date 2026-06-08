package egopidautotuneplugin;

import com.efiAnalytics.plugin.ApplicationPlugin;
import com.efiAnalytics.plugin.ecu.ControllerAccess;
import com.efiAnalytics.plugin.ecu.ControllerException;
import com.efiAnalytics.plugin.ecu.ControllerParameter;
import com.efiAnalytics.plugin.ecu.OutputChannel;
import com.efiAnalytics.plugin.ecu.OutputChannelClient;
import com.efiAnalytics.plugin.ecu.servers.ControllerParameterServer;

import javax.swing.*;
import javax.swing.border.TitledBorder;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * EGO (wideband O2 / lambda) PID Autotune Plugin for TunerStudio.
 *
 * Continuous relay-feedback loop. Start -> keeps cycling until Stop is pressed.
 *
 * Each iteration:
 *   1. Baseline  (BASELINE_MS) - observe AFR error at rest.
 *   2. Relay     (RELAY_MS)    - inflate KP by RELAY_KP_MULTIPLIER to force a
 *                                visible limit cycle; record oscillation data.
 *   3. Settle    (SETTLE_MS)   - restore best-so-far gains; record settling.
 *   4. Compute   - Astrom-Hagglund relay -> Ku, Tu -> Z-N PID rules scaled to
 *                  the 0-100 output range; Cohen-Coon cross-check.
 *   5. Write     - immediately write new gains to ECU; loop back to step 1.
 *
 * Gain scaling:
 *   Raw Ku from 4d/(pi*A) is dimensionless (~1.27 when d~A).
 *   Multiply by OUTPUT_SCALE (default 50) to map into the 0-100 correction
 *   output domain.  Tune OUTPUT_SCALE if results are still too low or too high.
 */
public class EgoPidAutotunePlugin extends JPanel implements ApplicationPlugin {

    private static final long serialVersionUID = 1L;

    private static final Logger LOG = Logger.getLogger(EgoPidAutotunePlugin.class.getName());

    // Algorithm constants
    private static final long   BASELINE_MS        = 20_000;
    private static final long   RELAY_MS            = 15_000;
    private static final long   SETTLE_MS           = 20_000;
    private static final long   SAMPLE_MS           = 100;
    private static final double RELAY_KP_MULTIPLIER = 4.0;
    private static final double RELAY_KP_MAX        = 200.0;
    private static final double MAX_GAIN            = 100.0;
    private static final double MIN_RELAY_AMP       = 0.05;
    /**
     * Scale factor: maps dimensionless Ku (AFR/AFR ~1.27) into the 0-100
     * correction output domain.  50 assumes ~100 units of output authority
     * per ~2 AFR units of authority.  Adjust if gains are still too low/high.
     */
    private static final double OUTPUT_SCALE        = 50.0;

    // ECU state
    private ControllerAccess          controllerAccess;
    private String                    ecuName;
    private ControllerParameterServer paramServer;

    private volatile double liveAfr        = 14.7;
    private volatile double liveCorrection = 0.0;
    private volatile double liveClt        = 0.0;
    private volatile double liveRpm        = 0.0;
    private volatile double liveLoad       = 0.0;

    private final AtomicBoolean            tuning         = new AtomicBoolean(false);
    private final AtomicBoolean            stopRequested  = new AtomicBoolean(false);
    private final AtomicInteger            iterationCount = new AtomicInteger(0);
    private       ScheduledExecutorService scheduler;
    private       ScheduledExecutorService refreshScheduler;

    private OutputChannelClient afrSubscriber;
    private OutputChannelClient corrSubscriber;
    private OutputChannelClient cltSubscriber;
    private OutputChannelClient rpmSubscriber;
    private OutputChannelClient loadSubscriber;

    private final AtomicReference<double[]> bestGains      = new AtomicReference<>(null);
    private volatile double  relayBaseKp, relayBaseKi, relayBaseKd;
    private volatile boolean relayBaseValid = false;

    // UI Step 1
    private final JButton   btnDump = new JButton("Dump ECU Values");
    private final JTextArea taDump  = new JTextArea(8, 60);

    // UI Step 2 config
    private final JTextField txtAfrChannel   = new JTextField("afr",           20);
    private final JTextField txtCorrChannel  = new JTextField("egoCorrection", 20);
    private final JTextField txtCltChannel   = new JTextField("coolant",       20);
    private final JTextField txtRpmChannel   = new JTextField("rpm",           20);
    private final JTextField txtLoadChannel  = new JTextField("map",           20);
    private final JTextField txtAfrTable     = new JTextField("afrTable",      20);
    private final JTextField txtAfrRpmAxis   = new JTextField("rpmBinsAFR",    20);
    private final JTextField txtAfrLoadAxis  = new JTextField("loadBinsAFR",   20);
    private final JTextField txtKpParam      = new JTextField("egoKP",         20);
    private final JTextField txtKiParam      = new JTextField("egoKI",         20);
    private final JTextField txtKdParam      = new JTextField("egoKD",         20);
    private final JTextField txtEgoTempParam = new JTextField("egoTemp",       20);

    // UI Step 3
    private final JLabel    lblLiveClt     = new JLabel("--- deg");
    private final JLabel    lblLiveAfr     = new JLabel("---");
    private final JLabel    lblTargetAfr   = new JLabel("---");
    private final JLabel    lblLiveCorr    = new JLabel("---");
    private final JLabel    lblEgoTempGate = new JLabel("--- deg  (not yet read)");
    private final JLabel    lblLiveRpm     = new JLabel("--- RPM");
    private final JLabel    lblLiveLoad    = new JLabel("--- (load)");
    private final JLabel    lblIteration   = new JLabel("Iteration: -");

    private final JButton      btnToggle       = new JButton("Start Autotune");
    private final JCheckBox    chkSkipTempGate = new JCheckBox("Skip temperature gate (debug only)");
    private final JProgressBar progressBar     = new JProgressBar(0, 100);
    private final JTextArea    taLog           = new JTextArea(12, 60);
    private final JLabel       lblKp           = new JLabel("---");
    private final JLabel       lblKi           = new JLabel("---");
    private final JLabel       lblKd           = new JLabel("---");

    // UI Step 4
    private final JTextField   fldKpLive    = new JTextField(8);
    private final JTextField   fldKiLive    = new JTextField(8);
    private final JTextField   fldKdLive    = new JTextField(8);
    private final JTextField   fldEgoTemp   = new JTextField(8);
    private final JTextField[] fldAfrTable  = new JTextField[16];
    private final JButton      btnReadTables  = new JButton("Read from ECU");
    private final JButton      btnWriteTables = new JButton("Write All to ECU");

    private final DecimalFormat df4 = new DecimalFormat("0.0000");
    private final DecimalFormat df2 = new DecimalFormat("0.00");
    private final DecimalFormat df1 = new DecimalFormat("0.0");

    public EgoPidAutotunePlugin() { buildUi(); }

    private void buildUi() {
        setLayout(new BorderLayout(6, 6));

        // Step 1: Dump
        JPanel pDump = new JPanel(new BorderLayout(4, 4));
        pDump.setBorder(titled("Step 1 - Dump ECU Channels & Parameters"));
        taDump.setEditable(false);
        taDump.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 11));
        pDump.add(new JScrollPane(taDump), BorderLayout.CENTER);
        btnDump.addActionListener(this::onDump);
        JPanel pDumpTop = new JPanel(new FlowLayout(FlowLayout.LEFT));
        pDumpTop.add(btnDump);
        pDumpTop.add(new JLabel("  <- use to find exact parameter/channel names for your ECU"));
        pDump.add(pDumpTop, BorderLayout.NORTH);

        // Step 2: Config
        JPanel pConfig = new JPanel(new GridBagLayout());
        pConfig.setBorder(titled("Step 2 - Configure Names (edit to match your ECU)"));
        GridBagConstraints gc = new GridBagConstraints();
        gc.insets = new Insets(3, 6, 3, 6);
        gc.anchor = GridBagConstraints.WEST;
        addConfigRow(pConfig, gc,  0, "AFR output channel (wideband):",                  txtAfrChannel);
        addConfigRow(pConfig, gc,  1, "EGO correction output channel:",                   txtCorrChannel);
        addConfigRow(pConfig, gc,  2, "Coolant temp output channel:",                     txtCltChannel);
        addConfigRow(pConfig, gc,  3, "RPM output channel:",                              txtRpmChannel);
        addConfigRow(pConfig, gc,  4, "Load output channel (map or tps):",                txtLoadChannel);
        addConfigRow(pConfig, gc,  5, "AFR target table param (2-D array):",              txtAfrTable);
        addConfigRow(pConfig, gc,  6, "AFR table RPM axis param:",                        txtAfrRpmAxis);
        addConfigRow(pConfig, gc,  7, "AFR table load axis param:",                       txtAfrLoadAxis);
        addConfigRow(pConfig, gc,  8, "egoKP parameter name:",                            txtKpParam);
        addConfigRow(pConfig, gc,  9, "egoKI parameter name:",                            txtKiParam);
        addConfigRow(pConfig, gc, 10, "egoKD parameter name:",                            txtKdParam);
        addConfigRow(pConfig, gc, 11, "egoTemp parameter name (CLT enable threshold):",   txtEgoTempParam);

        // Step 3: Live readout + continuous autotune
        JPanel pTune = new JPanel(new BorderLayout(4, 4));
        pTune.setBorder(titled("Step 3 - Live Readout & Continuous EGO Autotune"));

        JPanel pLiveBar = new JPanel(new FlowLayout(FlowLayout.LEFT, 12, 4));
        pLiveBar.setBorder(BorderFactory.createEtchedBorder());
        for (JLabel l : new JLabel[]{lblLiveClt, lblLiveAfr, lblTargetAfr,
                                      lblLiveCorr, lblLiveRpm, lblLiveLoad})
            l.setFont(l.getFont().deriveFont(Font.BOLD, 13f));
        pLiveBar.add(new JLabel("CLT:"));      pLiveBar.add(lblLiveClt);
        pLiveBar.add(sep());
        pLiveBar.add(new JLabel("AFR:"));      pLiveBar.add(lblLiveAfr);
        pLiveBar.add(new JLabel(" Target:"));  pLiveBar.add(lblTargetAfr);
        pLiveBar.add(sep());
        pLiveBar.add(new JLabel("Corr:"));     pLiveBar.add(lblLiveCorr);
        pLiveBar.add(sep());
        pLiveBar.add(new JLabel("RPM:"));      pLiveBar.add(lblLiveRpm);
        pLiveBar.add(new JLabel(" Load:"));    pLiveBar.add(lblLiveLoad);

        JPanel pGate = new JPanel(new FlowLayout(FlowLayout.LEFT, 10, 2));
        pGate.setBorder(BorderFactory.createEtchedBorder());
        pGate.add(new JLabel("EGO temp gate (egoTemp):"));
        pGate.add(lblEgoTempGate);
        chkSkipTempGate.setForeground(new Color(160, 0, 0));
        chkSkipTempGate.setToolTipText("Bypasses CLT >= egoTemp. Only use for bench testing.");
        pGate.add(chkSkipTempGate);

        progressBar.setStringPainted(true);
        progressBar.setString("idle");
        progressBar.setPreferredSize(new Dimension(280, 20));
        btnToggle.setFont(btnToggle.getFont().deriveFont(Font.BOLD));
        btnToggle.setForeground(new Color(0, 120, 0));
        btnToggle.addActionListener(this::onToggle);
        JPanel pControl = new JPanel(new FlowLayout(FlowLayout.LEFT));
        pControl.add(btnToggle);
        pControl.add(Box.createHorizontalStrut(10));
        pControl.add(lblIteration);
        pControl.add(Box.createHorizontalStrut(10));
        pControl.add(chkSkipTempGate);
        pControl.add(Box.createHorizontalStrut(16));
        pControl.add(progressBar);

        JPanel pResults = new JPanel(new FlowLayout(FlowLayout.LEFT, 12, 4));
        pResults.add(new JLabel("Best KP:")); pResults.add(lblKp);
        pResults.add(new JLabel("  KI:"));   pResults.add(lblKi);
        pResults.add(new JLabel("  KD:"));   pResults.add(lblKd);
        pResults.add(new JLabel("  (written to ECU after each iteration)"));

        taLog.setEditable(false);
        taLog.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 11));

        JPanel pLiveStack = new JPanel(new BorderLayout(2, 2));
        pLiveStack.add(pLiveBar, BorderLayout.NORTH);
        pLiveStack.add(pGate,   BorderLayout.SOUTH);
        JPanel pTuneNorth = new JPanel(new BorderLayout());
        pTuneNorth.add(pLiveStack, BorderLayout.NORTH);
        pTuneNorth.add(pControl,   BorderLayout.SOUTH);

        pTune.add(pTuneNorth,             BorderLayout.NORTH);
        pTune.add(new JScrollPane(taLog), BorderLayout.CENTER);
        pTune.add(pResults,               BorderLayout.SOUTH);

        JPanel pEditor = buildEditorPanel();

        JPanel pLeft = new JPanel(new BorderLayout(4, 4));
        JPanel pTop  = new JPanel(new BorderLayout());
        pTop.add(pDump,   BorderLayout.NORTH);
        pTop.add(pConfig, BorderLayout.SOUTH);
        pLeft.add(pTop,  BorderLayout.NORTH);
        pLeft.add(pTune, BorderLayout.CENTER);

        JSplitPane split = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, pLeft, pEditor);
        split.setResizeWeight(0.70);
        split.setOneTouchExpandable(true);
        add(split, BorderLayout.CENTER);
    }

    private Component sep() { return Box.createHorizontalStrut(10); }

    private JPanel buildEditorPanel() {
        JPanel p = new JPanel(new BorderLayout(6, 6)) {
            @Override public Dimension getPreferredSize() { return new Dimension(260, super.getPreferredSize().height); }
            @Override public Dimension getMinimumSize()   { return new Dimension(210, 0); }
        };
        p.setBorder(titled("Step 4 - Live Editor (reads & writes ECU directly)"));

        JPanel pBtns = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 4));
        pBtns.add(btnReadTables);
        pBtns.add(btnWriteTables);
        pBtns.add(new JLabel("  Write -> ECU RAM. Remember to Burn."));
        btnReadTables .addActionListener(e -> readTablesFromEcu());
        btnWriteTables.addActionListener(e -> writeTablesToEcu());
        p.add(pBtns, BorderLayout.NORTH);

        JPanel pAll = new JPanel();
        pAll.setLayout(new BoxLayout(pAll, BoxLayout.Y_AXIS));

        JPanel pPid = new JPanel(new GridBagLayout()) {
            @Override public Dimension getMaximumSize() {
                return new Dimension(Integer.MAX_VALUE, super.getPreferredSize().height); }
        };
        pPid.setBorder(titled("EGO PID Gains  (0-100 scale)"));
        GridBagConstraints gc = new GridBagConstraints();
        gc.insets = new Insets(4, 6, 4, 6);
        gc.anchor = GridBagConstraints.WEST;
        for (JTextField f : new JTextField[]{fldKpLive, fldKiLive, fldKdLive, fldEgoTemp})
            f.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
        gc.gridx=0; gc.gridy=0; pPid.add(new JLabel("KP  (egoKP):"),  gc); gc.gridx=1; pPid.add(fldKpLive,  gc);
        gc.gridx=0; gc.gridy=1; pPid.add(new JLabel("KI  (egoKI):"),  gc); gc.gridx=1; pPid.add(fldKiLive,  gc);
        gc.gridx=0; gc.gridy=2; pPid.add(new JLabel("KD  (egoKD):"),  gc); gc.gridx=1; pPid.add(fldKdLive,  gc);
        gc.gridx=0; gc.gridy=3; pPid.add(new JLabel("egoTemp (C):"),  gc); gc.gridx=1; pPid.add(fldEgoTemp, gc);
        gc.gridx=0; gc.gridy=4; gc.gridwidth=2;
        pPid.add(new JLabel("<html><i>Edit then click Write All to ECU</i></html>"), gc);

        JPanel pAfr = new JPanel(new BorderLayout(2, 2)) {
            @Override public Dimension getMaximumSize() {
                return new Dimension(Integer.MAX_VALUE, super.getPreferredSize().height); }
        };
        pAfr.setBorder(titled("AFR Target Table (first 16 cells, row-major)"));
        JPanel pAfrGrid = new JPanel(new GridLayout(0, 2, 2, 2));
        pAfrGrid.add(new JLabel("Cell #", JLabel.CENTER));
        pAfrGrid.add(new JLabel("Target AFR", JLabel.CENTER));
        for (int i = 0; i < fldAfrTable.length; i++) {
            fldAfrTable[i] = new JTextField("--", 6);
            fldAfrTable[i].setFont(new Font(Font.MONOSPACED, Font.PLAIN, 11));
            fldAfrTable[i].setHorizontalAlignment(JTextField.CENTER);
            fldAfrTable[i].setEnabled(false);
            pAfrGrid.add(new JLabel("" + i, JLabel.CENTER));
            pAfrGrid.add(fldAfrTable[i]);
        }
        pAfr.add(new JScrollPane(pAfrGrid), BorderLayout.CENTER);

        pAll.add(pPid);
        pAll.add(Box.createVerticalStrut(4));
        pAll.add(pAfr);
        pAll.add(Box.createVerticalGlue());
        p.add(new JScrollPane(pAll,
            JScrollPane.VERTICAL_SCROLLBAR_AS_NEEDED,
            JScrollPane.HORIZONTAL_SCROLLBAR_NEVER), BorderLayout.CENTER);
        return p;
    }

    private void addConfigRow(JPanel p, GridBagConstraints gc,
                              int row, String label, JTextField field) {
        gc.gridx = 0; gc.gridy = row; gc.weightx = 0; p.add(new JLabel(label), gc);
        gc.gridx = 1; gc.weightx = 1; p.add(field, gc);
    }

    private TitledBorder titled(String t) { return BorderFactory.createTitledBorder(t); }

    @Override public String     getIdName()            { return "egoPidAutotune"; }
    @Override public int        getPluginType()         { return PERSISTENT_DIALOG_PANEL; }
    @Override public String     getDisplayName()        { return "EGO PID Autotune"; }
    @Override public String     getDescription()        { return "Continuous relay-feedback autotune for EGO correction PID gains (0-100 scale)."; }
    @Override public String     getAuthor()             { return "Leon Pareike"; }
    @Override public String     getVersion()            { return "Beta 2.0"; }
    @Override public double     getRequiredPluginSpec() { return 1.0; }
    @Override public String     getHelpUrl()            { return null; }
    @Override public JComponent getPluginPanel()        { return this; }
    @Override public boolean    displayPlugin(String s) { return s != null && !s.isEmpty(); }
    @Override public boolean    isMenuEnabled()         { return true; }

    @Override
    public void initialize(ControllerAccess ca) {
        this.controllerAccess = ca;
        this.ecuName          = ca.getEcuConfigurationNames()[0];
        this.paramServer      = ca.getControllerParameterServer();

        afrSubscriber  = (ch, val) -> {
            liveAfr = val;
            SwingUtilities.invokeLater(() -> {
                lblLiveAfr.setText(df2.format(val));
                updateAfrColour(val);
                lblTargetAfr.setText(df2.format(safeReadAfrTarget()));
            });
        };
        corrSubscriber = (ch, val) -> {
            liveCorrection = val;
            SwingUtilities.invokeLater(() -> lblLiveCorr.setText(df2.format(val) + "%"));
        };
        cltSubscriber  = (ch, val) -> {
            liveClt = val;
            SwingUtilities.invokeLater(() -> updateCltDisplay(val));
        };
        rpmSubscriber  = (ch, val) -> {
            liveRpm = val;
            SwingUtilities.invokeLater(() -> lblLiveRpm.setText(df1.format(val) + " RPM"));
        };
        loadSubscriber = (ch, val) -> {
            liveLoad = val;
            SwingUtilities.invokeLater(() -> lblLiveLoad.setText(df1.format(val)));
        };

        subscribeChannels();
        readTablesFromEcu();

        refreshScheduler = Executors.newSingleThreadScheduledExecutor(r -> {
            Thread t = new Thread(r, "ego-table-refresh"); t.setDaemon(true); return t;
        });
        refreshScheduler.scheduleAtFixedRate(() -> {
            if (!tuning.get()) readTablesFromEcu();
        }, 5, 5, TimeUnit.SECONDS);

        log("Plugin initialized.  ECU: " + ecuName);
        log("OUTPUT_SCALE = " + OUTPUT_SCALE
            + "  (increase if gains are still too low; decrease if EGO oscillates wildly)");
        log("Click 'Dump ECU Values' to discover channel/parameter names for your ECU.");
    }

    private void subscribeChannels() {
        subscribe(txtAfrChannel.getText().trim(),  afrSubscriber,  "AFR");
        subscribe(txtCorrChannel.getText().trim(), corrSubscriber, "EGO correction");
        subscribe(txtCltChannel.getText().trim(),  cltSubscriber,  "CLT");
        subscribe(txtRpmChannel.getText().trim(),  rpmSubscriber,  "RPM");
        subscribe(txtLoadChannel.getText().trim(), loadSubscriber, "Load");
    }

    private void subscribe(String ch, OutputChannelClient sub, String label) {
        try {
            controllerAccess.getOutputChannelServer().subscribe(ecuName, ch, sub);
        } catch (ControllerException ex) {
            log("WARNING: cannot subscribe " + label + " channel '" + ch + "': " + ex.getMessage());
        }
    }

    private void updateCltDisplay(double clt) {
        lblLiveClt.setText(df1.format(clt) + " C");
        if      (clt < 60)  lblLiveClt.setForeground(new Color(0, 80, 200));
        else if (clt < 100) lblLiveClt.setForeground(new Color(0, 140, 0));
        else                lblLiveClt.setForeground(Color.RED);
    }

    private void updateAfrColour(double afr) {
        double err = Math.abs(afr - safeReadAfrTarget());
        if      (err < 0.3) lblLiveAfr.setForeground(new Color(0, 140, 0));
        else if (err < 0.8) lblLiveAfr.setForeground(new Color(180, 120, 0));
        else                lblLiveAfr.setForeground(Color.RED);
    }

    @Override
    public void close() {
        stopScheduler();
        if (refreshScheduler != null && !refreshScheduler.isShutdown())
            refreshScheduler.shutdownNow();
        if (controllerAccess != null) {
            for (OutputChannelClient s : new OutputChannelClient[]{
                    afrSubscriber, corrSubscriber, cltSubscriber, rpmSubscriber, loadSubscriber})
                try { controllerAccess.getOutputChannelServer().unsubscribe(s); } catch (Exception ignore) {}
        }
    }

    // Step 1 - Dump
    private void onDump(ActionEvent e) {
        if (controllerAccess == null) { taDump.setText("Not connected to ECU."); return; }
        StringBuilder sb = new StringBuilder(), sbSc = new StringBuilder(), sbArr = new StringBuilder();
        int chCnt = 0, scCnt = 0, arrCnt = 0;
        sb.append("=== OUTPUT CHANNELS ===\n");
        try {
            for (String ch : controllerAccess.getOutputChannelServer().getOutputChannels(ecuName)) {
                try {
                    OutputChannel oc = controllerAccess.getOutputChannelServer().getOutputChannel(ecuName, ch);
                    sb.append(String.format("  %-40s  units=%s\n", oc.getName(),
                        oc.getUnits() == null ? "" : oc.getUnits()));
                } catch (Exception ex) { sb.append("  ").append(ch).append(" (error)\n"); }
                chCnt++;
            }
            if (chCnt == 0) sb.append("  (none found)\n");
        } catch (Exception ex) { sb.append("  Error: ").append(ex.getMessage()).append("\n"); }
        for (String pn : paramServer.getParameterNames(ecuName)) {
            try {
                ControllerParameter cp = paramServer.getControllerParameter(ecuName, pn);
                if (ControllerParameter.PARAM_CLASS_SCALAR.equals(cp.getParamClass())) {
                    sbSc.append(String.format("  %-40s  value=%s\n", pn, df4.format(cp.getScalarValue())));
                    scCnt++;
                } else if (ControllerParameter.PARAM_CLASS_ARRAY.equals(cp.getParamClass())) {
                    double[] arr = flatten2D(cp.getArrayValues());
                    sbArr.append(arr.length > 0
                        ? String.format("  %-40s  cells=%d  first=%.2f  last=%.2f\n",
                            pn, arr.length, arr[0], arr[arr.length-1])
                        : String.format("  %-40s  cells=0\n", pn));
                    arrCnt++;
                }
            } catch (Exception ignored) {}
        }
        sb.append("\n=== SCALAR PARAMETERS ===\n").append(scCnt > 0 ? sbSc : "  (none)\n");
        sb.append("\n=== ARRAY / TABLE PARAMETERS ===\n").append(arrCnt > 0 ? sbArr : "  (none)\n");
        taDump.setText(sb.toString());
        taDump.setCaretPosition(0);
        log(String.format("Dump: %d channels, %d scalars, %d arrays.", chCnt, scCnt, arrCnt));
    }

    // Step 3 - Start / Stop toggle
    private void onToggle(ActionEvent e) {
        if (tuning.get()) {
            stopRequested.set(true);
            btnToggle.setEnabled(false);
            btnToggle.setText("Stopping...");
            log("Stop requested - finishing current phase then exiting.");
        } else {
            if (controllerAccess == null)                   { log("ERROR: no ECU connection.");      return; }
            if (!ControllerAccess.getInstance().isOnline()) { log("ERROR: TunerStudio not online."); return; }

            bestGains.set(null);
            relayBaseValid = false;
            stopRequested.set(false);
            iterationCount.set(0);
            tuning.set(true);

            btnToggle.setText("Stop Autotune");
            btnToggle.setForeground(new Color(160, 0, 0));
            lblKp.setText("---"); lblKi.setText("---"); lblKd.setText("---");
            setProgress(0, "Starting...");

            log("====== EGO Continuous Autotune Started ======");
            log(String.format("CLT=%.1fC  RPM=%.0f  Load=%.1f  AFR=%.2f",
                liveClt, liveRpm, liveLoad, liveAfr));
            log(String.format("OUTPUT_SCALE=%.1f  MAX_GAIN=%.0f  RELAY_KP_MAX=%.0f",
                OUTPUT_SCALE, MAX_GAIN, RELAY_KP_MAX));

            scheduler = Executors.newSingleThreadScheduledExecutor(r -> {
                Thread t = new Thread(r, "ego-autotune"); t.setDaemon(true); return t;
            });
            scheduler.execute(this::runContinuousLoop);
        }
    }

    // -------------------------------------------------------------------------
    // Continuous relay-feedback loop
    // -------------------------------------------------------------------------
    private void runContinuousLoop() {
        try {
            // Gate check
            double egoTempGate = safeReadScalar(txtEgoTempParam.getText().trim());
            final double etg = egoTempGate;
            SwingUtilities.invokeLater(() -> lblEgoTempGate.setText(df1.format(etg) + " C"));
            log(String.format("egoTemp gate=%.1fC  current CLT=%.1fC", egoTempGate, liveClt));
            if (!chkSkipTempGate.isSelected() && liveClt < egoTempGate) {
                log(String.format("ERROR: CLT (%.1fC) below egoTemp gate (%.1fC). "
                    + "Warm engine or tick Skip temperature gate.", liveClt, egoTempGate));
                return;
            }
            log(chkSkipTempGate.isSelected()
                ? "  WARNING: Temperature gate bypassed (debug mode)."
                : "  Temperature gate: PASSED");

            // Save original gains
            relayBaseKp = safeReadScalar(txtKpParam.getText().trim());
            relayBaseKi = safeReadScalar(txtKiParam.getText().trim());
            relayBaseKd = safeReadScalar(txtKdParam.getText().trim());
            relayBaseValid = true;
            log(String.format("Initial gains from ECU: KP=%.4f  KI=%.4f  KD=%.4f",
                relayBaseKp, relayBaseKi, relayBaseKd));
            bestGains.set(new double[]{relayBaseKp, relayBaseKi, relayBaseKd});

            // Main loop
            while (!stopRequested.get()) {
                int iter = iterationCount.incrementAndGet();
                final int fi = iter;
                SwingUtilities.invokeLater(() -> lblIteration.setText("Iteration: " + fi));
                log(String.format("---------------- Iteration %d ----------------", iter));

                double afrTarget = readAfrTarget(liveRpm, liveLoad);
                SwingUtilities.invokeLater(() -> lblTargetAfr.setText(df2.format(afrTarget)));
                log(String.format("  AFR target=%.2f  RPM=%.0f  Load=%.1f", afrTarget, liveRpm, liveLoad));

                // Phase 1 - Baseline
                if (stopRequested.get()) break;
                log(String.format("  Phase 1: Baseline (%.0f s)...", BASELINE_MS / 1000.0));
                setProgress(5, "Iter " + iter + ": Baseline...");
                List<double[]> baseline = collectAfrError(BASELINE_MS, afrTarget, 5, 30,
                    "Iter " + iter + ": Baseline %.0f/%.0f s");
                double baseMean = mean(baseline);
                double baseStd  = stdDev(baseline, baseMean);
                log(String.format("    mean err=%.4f  stdDev=%.4f", baseMean, baseStd));

                // Phase 2 - Relay excitation
                if (stopRequested.get()) break;
                double[] cur = bestGains.get();
                double probeKp = Math.min(cur[0] * RELAY_KP_MULTIPLIER, RELAY_KP_MAX);
                if (probeKp < 1e-3) probeKp = 5.0;   // seed for 0-100 scale when KP is zero
                log(String.format("  Phase 2: Relay (%.0f s)  probeKP=%.4f...", RELAY_MS / 1000.0, probeKp));
                setProgress(30, "Iter " + iter + ": Relay...");
                paramServer.updateParameter(ecuName, txtKpParam.getText().trim(), probeKp);

                List<double[]> relayData = collectAfrError(RELAY_MS, afrTarget, 30, 55,
                    "Iter " + iter + ": Relay %.0f/%.0f s");

                // Restore best-so-far gains immediately
                paramServer.updateParameter(ecuName, txtKpParam.getText().trim(), cur[0]);
                paramServer.updateParameter(ecuName, txtKiParam.getText().trim(), cur[1]);
                paramServer.updateParameter(ecuName, txtKdParam.getText().trim(), cur[2]);
                log("    Relay done - best gains restored.");

                // Phase 3 - Settling
                if (stopRequested.get()) break;
                log(String.format("  Phase 3: Settle (%.0f s)...", SETTLE_MS / 1000.0));
                setProgress(57, "Iter " + iter + ": Settle...");
                List<double[]> settleData = collectAfrError(SETTLE_MS, afrTarget, 57, 88,
                    "Iter " + iter + ": Settle %.0f/%.0f s");
                double settleMean = mean(settleData);
                double settleStd  = stdDev(settleData, settleMean);
                log(String.format("    settle mean=%.4f  stdDev=%.4f", settleMean, settleStd));

                // Phase 4 - Compute
                if (relayData.size() < 20) {
                    log("    WARNING: too few relay samples - skipping.");
                    continue;
                }
                setProgress(90, "Iter " + iter + ": Computing...");
                log("  Phase 4: Compute");

                RelayResult relay = relayAnalysis(relayData, 0.0);

                // Scale raw Ku into the 0-100 output domain
                // Raw Ku = 4d/(pi*A) ~= 1.27  (dimensionless, AFR/AFR)
                // Scaled Ku = Ku_raw * OUTPUT_SCALE  (correction-units per AFR-unit)
                double kuScaled = relay.Ku * OUTPUT_SCALE;
                log(String.format("    A=%.4f AFR  Tu=%.3f s  Ku_raw=%.6f  Ku_scaled=%.4f  (x%.1f)",
                    relay.amplitude, relay.Tu, relay.Ku, kuScaled, OUTPUT_SCALE));

                if (relay.amplitude < MIN_RELAY_AMP) {
                    log("    WARNING: relay amplitude too small - EGO may already be converged.");
                    log("             Gains not updated this iteration.");
                    setProgress(100, "Iter " + iter + " - stable, no update");
                    continue;
                }

                // Ziegler-Nichols relay (primary)
                double kp_zn = 0.60  * kuScaled;
                double ki_zn = 1.20  * kuScaled / relay.Tu;
                double kd_zn = 0.075 * kuScaled * relay.Tu;
                log(String.format("    Z-N:        KP=%.4f  KI=%.4f  KD=%.4f", kp_zn, ki_zn, kd_zn));

                // Cohen-Coon cross-check (FOPDT fit to settle data, OUTPUT_SCALE applied to K)
                FopdtResult fopdt = fitFopdt(settleData, settleMean, OUTPUT_SCALE);
                double ratio  = fopdt.theta / fopdt.tau;
                double kp_cc  = (fopdt.tau / (fopdt.K * fopdt.theta)) * (4.0/3.0 + ratio/4.0);
                double tiCC   = fopdt.tau * (32.0 + 6.0*ratio) / (13.0 + 8.0*ratio);
                double ki_cc  = kp_cc / tiCC;
                double kd_cc  = kp_cc * (fopdt.tau * 4.0 / (11.0 + 2.0*ratio));
                log(String.format("    Cohen-Coon: KP=%.4f  KI=%.4f  KD=%.4f", kp_cc, ki_cc, kd_cc));

                // Z-N when amplitude is large; CC when marginal
                boolean useZN = relay.amplitude > MIN_RELAY_AMP * 3.0;
                double kp = useZN ? kp_zn : kp_cc;
                double ki = useZN ? ki_zn : ki_cc;
                double kd = useZN ? kd_zn : kd_cc;
                log("    Method: " + (useZN ? "Ziegler-Nichols relay" : "Cohen-Coon"));

                // Clamp to 0-100 range
                kp = clamp(kp, 0.1, MAX_GAIN);
                ki = clamp(ki, 0.1, MAX_GAIN);
                kd = clamp(kd, 0.0, MAX_GAIN);
                log(String.format("    Final:      KP=%.4f  KI=%.4f  KD=%.4f", kp, ki, kd));

                // Write immediately
                if (stopRequested.get()) break;
                paramServer.updateParameter(ecuName, txtKpParam.getText().trim(), kp);
                paramServer.updateParameter(ecuName, txtKiParam.getText().trim(), ki);
                paramServer.updateParameter(ecuName, txtKdParam.getText().trim(), kd);
                bestGains.set(new double[]{kp, ki, kd});

                final double fKp = kp, fKi = ki, fKd = kd;
                SwingUtilities.invokeLater(() -> {
                    lblKp.setText(df4.format(fKp));
                    lblKi.setText(df4.format(fKi));
                    lblKd.setText(df4.format(fKd));
                    fldKpLive.setText(df4.format(fKp));
                    fldKiLive.setText(df4.format(fKi));
                    fldKdLive.setText(df4.format(fKd));
                });
                log(String.format("    Gains written to ECU. Iteration %d complete.", iter));
                setProgress(100, "Iter " + iter + " done - looping...");
                Thread.sleep(2_000);
            }

            log("====== Autotune stopped after " + iterationCount.get() + " iteration(s). ======");
            log("Remember to Burn to flash if you are happy with the gains.");

        } catch (InterruptedException ie) {
            Thread.currentThread().interrupt();
            log("Autotune thread interrupted.");
            safeRestoreRelayBase();
        } catch (Exception ex) {
            String msg = ex.getMessage() != null ? ex.getMessage() : "(no message)";
            Throwable ca = ex.getCause();
            log("ERROR [" + ex.getClass().getSimpleName() + "] " + msg
                + (ca != null ? "  caused by: " + ca.getClass().getSimpleName()
                    + ": " + ca.getMessage() : ""));
            LOG.log(Level.SEVERE, "EGO autotune error", ex);
            safeRestoreRelayBase();
        } finally {
            tuning.set(false);
            stopRequested.set(false);
            SwingUtilities.invokeLater(() -> {
                btnToggle.setText("Start Autotune");
                btnToggle.setForeground(new Color(0, 120, 0));
                btnToggle.setEnabled(true);
                setProgress(0, "Stopped");
                readTablesFromEcu();
            });
            stopScheduler();
        }
    }

    private void safeRestoreRelayBase() {
        if (paramServer == null || !relayBaseValid) return;
        try {
            paramServer.updateParameter(ecuName, txtKpParam.getText().trim(), relayBaseKp);
            paramServer.updateParameter(ecuName, txtKiParam.getText().trim(), relayBaseKi);
            paramServer.updateParameter(ecuName, txtKdParam.getText().trim(), relayBaseKd);
            log("  Original gains restored after error.");
        } catch (Exception ex) {
            log("  WARNING: could not restore gains: " + ex.getMessage());
        }
    }

    // Sample collection (breaks early if stop requested)
    private List<double[]> collectAfrError(long durationMs, double afrTarget,
                                            int pctStart, int pctEnd,
                                            String fmtMsg) throws InterruptedException {
        List<double[]> samples = new ArrayList<>();
        long start = System.currentTimeMillis();
        long end   = start + durationMs;
        while (System.currentTimeMillis() < end && !stopRequested.get()) {
            double t = (System.currentTimeMillis() - start) / 1000.0;
            samples.add(new double[]{t, liveAfr - afrTarget});
            Thread.sleep(SAMPLE_MS);
            double frac = (System.currentTimeMillis() - start) / (double) durationMs;
            setProgress(Math.min(pctEnd, pctStart + (int)(frac * (pctEnd - pctStart))),
                String.format(fmtMsg, t, durationMs / 1000.0));
        }
        return samples;
    }

    // Relay analysis (Astrom-Hagglund)
    private static final class RelayResult {
        final double amplitude, Tu, Ku;
        RelayResult(double a, double tu, double ku) { amplitude=a; Tu=tu; Ku=ku; }
    }

    private RelayResult relayAnalysis(List<double[]> samples, double centerMean) {
        double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        for (double[] s : samples) { if (s[1] < min) min=s[1]; if (s[1] > max) max=s[1]; }
        double A  = (max - min) / 2.0;
        double d  = Math.max(A, MIN_RELAY_AMP);
        double Tu = estimateOscPeriod(samples, centerMean);
        // Raw Ku is dimensionless (~1.27). Caller multiplies by OUTPUT_SCALE.
        double Ku = (A > MIN_RELAY_AMP) ? (4.0 * d) / (Math.PI * A) : 0.0;
        return new RelayResult(A, Tu, Ku);
    }

    private static double estimateOscPeriod(List<double[]> samples, double mean) {
        List<Double> xTimes = new ArrayList<>();
        boolean above = samples.get(0)[1] > mean;
        for (int i = 1; i < samples.size(); i++) {
            boolean nowAbove = samples.get(i)[1] > mean;
            if (nowAbove != above) { xTimes.add(samples.get(i)[0]); above = nowAbove; }
        }
        if (xTimes.size() < 2) return 3.0;
        double sumGaps = 0;
        for (int i = 1; i < xTimes.size(); i++) sumGaps += xTimes.get(i) - xTimes.get(i-1);
        return clamp(2.0 * (sumGaps / (xTimes.size() - 1)), 0.3, 30.0);
    }

    // FOPDT fit (Sundaresan-Krishnaswamy 28%/63% method)
    private static final class FopdtResult {
        final double K, tau, theta;
        FopdtResult(double K, double tau, double theta) { this.K=K; this.tau=tau; this.theta=theta; }
    }

    private FopdtResult fitFopdt(List<double[]> samples, double baseErr, double outputScale) {
        if (samples.isEmpty()) return new FopdtResult(outputScale, 2.0, 0.5);
        int    tail  = Math.max(1, samples.size() / 5);
        double ssErr = 0;
        for (int i = samples.size() - tail; i < samples.size(); i++) ssErr += samples.get(i)[1];
        ssErr /= tail;
        double deltaErr = ssErr - baseErr;
        double min = Double.MAX_VALUE, max = -Double.MAX_VALUE;
        for (double[] s : samples) { if (s[1] < min) min=s[1]; if (s[1] > max) max=s[1]; }
        double stepMag = Math.max(MIN_RELAY_AMP, (max - min) / 2.0);
        // K scaled to 0-100 output units per AFR unit
        double K  = (deltaErr / stepMag) * outputScale;
        double t1 = findCrossing(samples, baseErr + 0.283 * deltaErr, deltaErr > 0);
        double t2 = findCrossing(samples, baseErr + 0.632 * deltaErr, deltaErr > 0);
        double tau   = Math.max(0.1,  1.5 * (t2 - t1));
        double theta = Math.max(0.05, Math.abs(t2 - 1.5 * (t2 - t1)));
        return new FopdtResult(K == 0 ? outputScale : K, tau, theta);
    }

    private double findCrossing(List<double[]> s, double target, boolean rising) {
        for (double[] p : s) {
            if (rising  && p[1] >= target) return p[0];
            if (!rising && p[1] <= target) return p[0];
        }
        return s.get(s.size() / 2)[0];
    }

    // AFR target lookup
    private double readAfrTarget(double rpm, double load) {
        String tableName = txtAfrTable.getText().trim();
        if (tableName.isEmpty()) return liveAfr;
        try {
            double[] rpmBins  = loadAxis(txtAfrRpmAxis.getText().trim());
            double[] loadBins = loadAxis(txtAfrLoadAxis.getText().trim());
            double[][] arr = paramServer.getControllerParameter(ecuName, tableName).getArrayValues();
            if (arr == null || arr.length == 0) return liveAfr;
            int rpmIdx  = (rpmBins  != null) ? findBin(rpmBins,  rpm)  : 0;
            int loadIdx = (loadBins != null) ? findBin(loadBins, load) : 0;
            if (loadIdx < arr.length && rpmIdx < arr[0].length)
                return arr[loadIdx][rpmIdx];
            double[] flat = flatten2D(arr);
            return flat[Math.min(rpmIdx, flat.length - 1)];
        } catch (Exception ex) { return liveAfr; }
    }

    private double safeReadAfrTarget() {
        try { return readAfrTarget(liveRpm, liveLoad); } catch (Exception ex) { return liveAfr; }
    }

    // Read / Write tables
    private void readTablesFromEcu() {
        if (paramServer == null) return;
        btnReadTables.setEnabled(false);
        new Thread(() -> {
            try {
                double kpVal  = safeReadScalar(txtKpParam.getText().trim());
                double kiVal  = safeReadScalar(txtKiParam.getText().trim());
                double kdVal  = safeReadScalar(txtKdParam.getText().trim());
                double etVal  = safeReadScalar(txtEgoTempParam.getText().trim());
                double[] afrFlat = loadAxis(txtAfrTable.getText().trim());
                SwingUtilities.invokeLater(() -> {
                    fldKpLive .setText(df4.format(kpVal));
                    fldKiLive .setText(df4.format(kiVal));
                    fldKdLive .setText(df4.format(kdVal));
                    fldEgoTemp.setText(df1.format(etVal));
                    lblEgoTempGate.setText(df1.format(etVal) + " C");
                    if (afrFlat != null) {
                        int n = Math.min(afrFlat.length, fldAfrTable.length);
                        for (int i = 0; i < fldAfrTable.length; i++) {
                            if (i < n) { fldAfrTable[i].setText(df2.format(afrFlat[i])); fldAfrTable[i].setEnabled(true); }
                            else       { fldAfrTable[i].setText(""); fldAfrTable[i].setEnabled(false); }
                        }
                    }
                    btnReadTables.setEnabled(true);
                });
            } catch (Exception ex) {
                SwingUtilities.invokeLater(() -> { log("ERROR reading tables: " + ex.getMessage()); btnReadTables.setEnabled(true); });
            }
        }, "ego-table-read").start();
    }

    private void writeTablesToEcu() {
        if (paramServer == null) return;
        if (JOptionPane.showConfirmDialog(this,
            "Write all edited values to ECU RAM? (Remember to Burn afterwards.)",
            "Confirm Write", JOptionPane.YES_NO_OPTION) != JOptionPane.YES_OPTION) return;
        btnWriteTables.setEnabled(false);
        new Thread(() -> {
            int errors = 0;
            errors += writeScalarField(fldKpLive,  txtKpParam.getText().trim(),      "egoKP");
            errors += writeScalarField(fldKiLive,  txtKiParam.getText().trim(),      "egoKI");
            errors += writeScalarField(fldKdLive,  txtKdParam.getText().trim(),      "egoKD");
            errors += writeScalarField(fldEgoTemp, txtEgoTempParam.getText().trim(), "egoTemp");
            errors += writeFieldsToAfrTable(fldAfrTable, txtAfrTable.getText().trim());
            final int fe = errors;
            SwingUtilities.invokeLater(() -> {
                if (fe == 0) log("All values written to ECU. Remember to Burn!");
                else         log("Write complete with " + fe + " error(s).");
                btnWriteTables.setEnabled(true);
            });
        }, "ego-table-write").start();
    }

    private int writeFieldsToAfrTable(JTextField[] fields, String paramName) {
        if (paramName.isEmpty()) return 0;
        try {
            ControllerParameter cp  = paramServer.getControllerParameter(ecuName, paramName);
            double[][]          arr = cp.getArrayValues();
            if (arr == null || arr.length == 0) return 0;
            int idx = 0;
            outer:
            for (int r = 0; r < arr.length; r++) {
                for (int c = 0; c < arr[r].length; c++) {
                    if (idx >= fields.length) break outer;
                    if (!fields[idx].isEnabled() || fields[idx].getText().trim().isEmpty()) { idx++; continue; }
                    try { arr[r][c] = Double.parseDouble(fields[idx].getText().trim().replace(',', '.')); }
                    catch (NumberFormatException ignored) {}
                    idx++;
                }
            }
            paramServer.updateParameter(ecuName, paramName, arr);
            log("  AFR table (" + paramName + ") written OK.");
            return 0;
        } catch (Exception ex) { log("  ERROR writing AFR table: " + ex.getMessage()); return 1; }
    }

    private int writeScalarField(JTextField field, String paramName, String label) {
        if (paramName.isEmpty() || field.getText().trim().isEmpty()) return 0;
        try {
            double val = Double.parseDouble(field.getText().trim().replace(',', '.'));
            paramServer.updateParameter(ecuName, paramName, val);
            log("  " + label + " (" + paramName + ") = " + val + " OK.");
            return 0;
        } catch (Exception ex) { log("  ERROR writing " + label + ": " + ex.getMessage()); return 1; }
    }

    // ECU helpers
    private double[] loadAxis(String paramName) {
        if (paramName == null || paramName.isEmpty()) return null;
        try {
            ControllerParameter cp = paramServer.getControllerParameter(ecuName, paramName);
            if (ControllerParameter.PARAM_CLASS_ARRAY.equals(cp.getParamClass()))
                return flatten2D(cp.getArrayValues());
        } catch (Exception ex) { /* return null */ }
        return null;
    }

    private int findBin(double[] axis, double val) {
        if (axis == null || axis.length == 0) return 0;
        if (val <= axis[0])               return 0;
        if (val >= axis[axis.length - 1]) return axis.length - 1;
        for (int i = 0; i < axis.length - 1; i++)
            if (val >= axis[i] && val < axis[i + 1]) return i;
        return axis.length - 1;
    }

    private double safeReadScalar(String name) {
        try { return paramServer.getControllerParameter(ecuName, name).getScalarValue(); }
        catch (Exception ex) { return 0.0; }
    }

    private static double[] flatten2D(double[][] arr2d) {
        if (arr2d == null || arr2d.length == 0) return new double[0];
        int total = 0;
        for (double[] row : arr2d) total += row.length;
        double[] flat = new double[total];
        int i = 0;
        for (double[] row : arr2d) for (double v : row) flat[i++] = v;
        return flat;
    }

    private static double mean(List<double[]> s) {
        double sum = 0; for (double[] p : s) sum += p[1]; return s.isEmpty() ? 0 : sum / s.size();
    }

    private static double stdDev(List<double[]> s, double mean) {
        double sum2 = 0; for (double[] p : s) sum2 += (p[1]-mean)*(p[1]-mean);
        return s.isEmpty() ? 0 : Math.sqrt(sum2 / s.size());
    }

    private void setProgress(int pct, String text) {
        SwingUtilities.invokeLater(() -> { progressBar.setValue(pct); progressBar.setString(text); });
    }

    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }

    private void stopScheduler() {
        if (scheduler != null && !scheduler.isShutdown()) scheduler.shutdownNow();
    }

    private void log(String msg) {
        SwingUtilities.invokeLater(() -> {
            taLog.append(msg + "\n");
            taLog.setCaretPosition(taLog.getDocument().getLength());
        });
        LOG.info(msg);
    }

    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame f = new JFrame("EGO PID Autotune - preview");
            f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            f.getContentPane().add(new EgoPidAutotunePlugin());
            f.pack(); f.setLocationRelativeTo(null); f.setVisible(true);
        });
    }
}