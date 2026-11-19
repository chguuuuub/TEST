package com.example.collision

import android.content.Context
import android.graphics.*
import android.os.Bundle
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.widget.*
import androidx.appcompat.app.AppCompatActivity
import kotlin.math.*
import kotlin.random.Random

// ==================== 2D 向量数学工具 ====================
data class Vec2(var x: Float, var y: Float) {
    operator fun plus(o: Vec2) = Vec2(x + o.x, y + o.y)
    operator fun minus(o: Vec2) = Vec2(x - o.x, y - o.y)
    operator fun times(s: Float) = Vec2(x * s, y * s)
    operator fun div(s: Float) = Vec2(x / s, y / s)
    fun dot(o: Vec2) = x * o.x + y * o.y
    fun cross(o: Vec2) = x * o.y - y * o.x
    fun lenSq() = x * x + y * y
    fun len() = sqrt(lenSq())
    fun norm(): Vec2 {
        val l = len()
        return if (l > 1e-6f) Vec2(x / l, y / l) else Vec2(0f, 0f)
    }
    fun perp() = Vec2(-y, x)
    fun rotate(rad: Float): Vec2 {
        val c = cos(rad)
        val s = sin(rad)
        return Vec2(x * c - y * s, x * s + y * c)
    }
}

// ==================== 形状枚举 ====================
enum class BodyShapeType(val label: String) {
    REGULAR_POLY("正多边形"),
    IRREGULAR_POLY("非正多边形"),
    CIRCLE("圆"),
    ELLIPSE("椭圆")
}

enum class BoundaryType(val label: String) {
    CIRCLE("圆形外框"),
    ELLIPSE("椭圆外框"),
    HEXAGON("正六边形外框"),
    SQUARE("正方形外框")
}

// ==================== 刚体对象 ====================
class RigidBody(
    val id: Int,
    var shapeType: BodyShapeType,
    var sides: Int,          // 边数（正多边形/非正多边形有效）
    var sideLength: Float,   // 基准边长/半径参数（像素）
    var color: Int,
    var pos: Vec2,
    var vel: Vec2,
    var angle: Float = 0f,
    var angularVel: Float = 0f
) {
    // 局部坐标系下的凸多边形顶点（已中心化，质心在 (0,0)）
    var localVertices: List<Vec2> = emptyList()
    var mass: Float = 1f
    var invMass: Float = 1f
    var inertia: Float = 1f
    var invInertia: Float = 1f
    var boundingRadius: Float = 50f

    init {
        rebuildGeometry()
    }

    fun rebuildGeometry() {
        val raw = mutableListOf<Vec2>()
        when (shapeType) {
            BodyShapeType.REGULAR_POLY -> {
                val n = sides.coerceIn(3, 12)
                // 由边长反推外接圆半径: R = a / (2 * sin(pi / n))
                val r = (sideLength / (2.0 * sin(PI / n))).toFloat().coerceIn(25f, 160f)
                for (i in 0 until n) {
                    val theta = (2.0 * PI * i / n).toFloat()
                    raw.add(Vec2(r * cos(theta), r * sin(theta)))
                }
            }
            BodyShapeType.IRREGULAR_POLY -> {
                val n = sides.coerceIn(3, 10)
                val baseR = (sideLength / (2.0 * sin(PI / n))).toFloat().coerceIn(30f, 150f)
                // 使用固定种子偏移生成确定性凸非正多边形（保证凸性以便 SAT 精确求解）
                val rng = Random(id * 97 + n * 13 + sideLength.toInt())
                val angles = (0 until n).map { i ->
                    val step = (2.0 * PI / n).toFloat()
                    i * step + rng.nextFloat() * step * 0.35f
                }.sorted()
                for (theta in angles) {
                    val scale = 0.72f + rng.nextFloat() * 0.56f
                    raw.add(Vec2(baseR * scale * cos(theta), baseR * scale * sin(theta)))
                }
            }
            BodyShapeType.CIRCLE -> {
                val r = sideLength.coerceIn(25f, 140f)
                val segments = 28
                for (i in 0 until segments) {
                    val theta = (2.0 * PI * i / segments).toFloat()
                    raw.add(Vec2(r * cos(theta), r * sin(theta)))
                }
            }
            BodyShapeType.ELLIPSE -> {
                val rx = sideLength.coerceIn(30f, 150f)
                val ry = (rx * 0.62f).coerceAtLeast(20f)
                val segments = 28
                for (i in 0 until segments) {
                    val theta = (2.0 * PI * i / segments).toFloat()
                    raw.add(Vec2(rx * cos(theta), ry * sin(theta)))
                }
            }
        }

        // 计算多边形面积与质心并归零
        var area2 = 0f
        var cx = 0f
        var cy = 0f
        for (i in raw.indices) {
            val p1 = raw[i]
            val p2 = raw[(i + 1) % raw.size]
            val cross = p1.cross(p2)
            area2 += cross
            cx += (p1.x + p2.x) * cross
            cy += (p1.y + p2.y) * cross
        }
        val area = abs(area2) * 0.5f
        val centroid = if (abs(area2) > 1e-5f) Vec2(cx / (3f * area2), cy / (3f * area2)) else Vec2(0f, 0f)
        localVertices = raw.map { it - centroid }

        // 质量与转动惯量（均匀密度）
        mass = (area / 1000f).coerceAtLeast(0.5f)
        invMass = 1f / mass

        var denom = 0f
        var numer = 0f
        var maxR = 0f
        for (i in localVertices.indices) {
            val p1 = localVertices[i]
            val p2 = localVertices[(i + 1) % localVertices.size]
            val cross = abs(p1.cross(p2))
            val term = p1.dot(p1) + p1.dot(p2) + p2.dot(p2)
            numer += cross * term
            denom += cross
            maxR = max(maxR, p1.len())
        }
        inertia = if (denom > 1e-5f) (mass / 6f) * (numer / denom) else mass * 1000f
        invInertia = 1f / inertia.coerceAtLeast(10f)
        boundingRadius = maxR
    }

    fun getWorldVertices(): List<Vec2> {
        return localVertices.map { it.rotate(angle) + pos }
    }
}

// ==================== 物理引擎与画布视图 ====================
class PhysicsSimulationView(context: Context) : View(context) {

    val bodies = mutableListOf<RigidBody>()
    var boundaryType: BoundaryType = BoundaryType.CIRCLE
    var restitution: Float = 0.96f // 弹性恢复系数（接近1保持活力，略小于1防止数值爆炸）

    private val bodyFillPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply { style = Paint.Style.FILL }
    private val bodyStrokePaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 3f
        color = Color.WHITE
    }
    private val markerPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 3f
        color = Color.argb(180, 255, 255, 255)
    }
    private val boundaryPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.STROKE
        strokeWidth = 6f
        color = Color.parseColor("#38BDF8")
    }
    private val boundaryBgPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        style = Paint.Style.FILL
        color = Color.parseColor("#0F172A")
    }

    private var lastTimeNs: Long = 0L
    private var center = Vec2(500f, 500f)
    private var boundaryRx = 420f
    private var boundaryRy = 420f
    private var boundaryWallVertices: List<Vec2> = emptyList()

    private val frameRunnable = object : Runnable {
        override fun run() {
            val now = System.nanoTime()
            if (lastTimeNs != 0L) {
                val dt = ((now - lastTimeNs) / 1_000_000_000f).coerceIn(0.001f, 0.033f)
                stepPhysics(dt)
            }
            lastTimeNs = now
            invalidate()
            postOnAnimation(this)
        }
    }

    init {
        setBackgroundColor(Color.parseColor("#020617"))
        initDefaultBodies(3)
    }

    override fun onAttachedToWindow() {
        super.onAttachedToWindow()
        lastTimeNs = System.nanoTime()
        postOnAnimation(frameRunnable)
    }

    override fun onDetachedFromWindow() {
        removeCallbacks(frameRunnable)
        super.onDetachedFromWindow()
    }

    override fun onSizeChanged(w: Int, h: Int, oldw: Int, oldh: Int) {
        super.onSizeChanged(w, h, oldw, oldh)
        center = Vec2(w * 0.5f, h * 0.5f)
        val minDim = min(w, h) * 0.44f
        boundaryRx = minDim
        boundaryRy = if (boundaryType == BoundaryType.ELLIPSE) minDim * 0.72f else minDim
        rebuildBoundary()
        repositionBodiesSafely()
    }

    fun setBoundary(type: BoundaryType) {
        boundaryType = type
        if (width > 0 && height > 0) {
            val minDim = min(width, height) * 0.44f
            boundaryRx = minDim
            boundaryRy = if (boundaryType == BoundaryType.ELLIPSE) minDim * 0.72f else minDim
            rebuildBoundary()
            repositionBodiesSafely()
        }
    }

    private fun rebuildBoundary() {
        val pts = mutableListOf<Vec2>()
        when (boundaryType) {
            BoundaryType.CIRCLE -> {
                val segs = 64
                for (i in 0 until segs) {
                    val t = (2.0 * PI * i / segs).toFloat()
                    pts.add(Vec2(center.x + boundaryRx * cos(t), center.y + boundaryRx * sin(t)))
                }
            }
            BoundaryType.ELLIPSE -> {
                val segs = 64
                for (i in 0 until segs) {
                    val t = (2.0 * PI * i / segs).toFloat()
                    pts.add(Vec2(center.x + boundaryRx * cos(t), center.y + boundaryRy * sin(t)))
                }
            }
            BoundaryType.HEXAGON -> {
                for (i in 0 until 6) {
                    val t = (2.0 * PI * i / 6.0).toFloat()
                    pts.add(Vec2(center.x + boundaryRx * cos(t), center.y + boundaryRx * sin(t)))
                }
            }
            BoundaryType.SQUARE -> {
                val r = boundaryRx * 0.85f
                pts.add(Vec2(center.x - r, center.y - r))
                pts.add(Vec2(center.x + r, center.y - r))
                pts.add(Vec2(center.x + r, center.y + r))
                pts.add(Vec2(center.x - r, center.y + r))
            }
        }
        boundaryWallVertices = pts
    }

    fun initDefaultBodies(count: Int) {
        bodies.clear()
        val presetColors = listOf(
            Color.parseColor("#F43F5E"),
            Color.parseColor("#10B981"),
            Color.parseColor("#F59E0B"),
            Color.parseColor("#8B5CF6")
        )
        val presetShapes = listOf(
            BodyShapeType.REGULAR_POLY,
            BodyShapeType.IRREGULAR_POLY,
            BodyShapeType.CIRCLE,
            BodyShapeType.ELLIPSE
        )
        val presetSides = listOf(3, 5, 6, 4)

        val cx = if (width > 0) width * 0.5f else 500f
        val cy = if (height > 0) height * 0.5f else 500f
        val spread = 130f

        for (i in 0 until count.coerceIn(2, 4)) {
            val ang = (2.0 * PI * i / count).toFloat()
            val p = Vec2(cx + spread * cos(ang), cy + spread * sin(ang))
            val speed = 260f + i * 35f
            val vAng = ang + 2.1f
            bodies.add(
                RigidBody(
                    id = i,
                    shapeType = presetShapes[i % presetShapes.size],
                    sides = presetSides[i % presetSides.size],
                    sideLength = 75f,
                    color = presetColors[i % presetColors.size],
                    pos = p,
                    vel = Vec2(speed * cos(vAng), speed * sin(vAng)),
                    angularVel = (if (i % 2 == 0) 1f else -1f) * 1.2f
                )
            )
        }
    }

    fun repositionBodiesSafely() {
        val count = bodies.size
        val spread = min(boundaryRx, boundaryRy) * 0.42f
        bodies.forEachIndexed { i, b ->
            val ang = (2.0 * PI * i / count).toFloat()
            b.pos = Vec2(center.x + spread * cos(ang), center.y + spread * sin(ang))
        }
    }

    // ==================== 物理步进（子步迭代提升精度） ====================
    private fun stepPhysics(dt: Float) {
        val subSteps = 6
        val subDt = dt / subSteps
        repeat(subSteps) {
            // 1. 积分位置与角度
            for (b in bodies) {
                b.pos += b.vel * subDt
                b.angle += b.angularVel * subDt
            }

            // 2. 多边形与外框碰撞
            for (b in bodies) {
                resolveBoundaryCollision(b)
            }

            // 3. 多边形之间两两碰撞 (SAT)
            for (i in 0 until bodies.size) {
                for (j in i + 1 until bodies.size) {
                    resolveBodyPairCollision(bodies[i], bodies[j])
                }
            }
        }
    }

    // ==================== 外框内壁碰撞求解 ====================
    private fun resolveBoundaryCollision(b: RigidBody) {
        if (boundaryWallVertices.size < 3) return
        val worldVerts = b.getWorldVertices()
        val wallCount = boundaryWallVertices.size

        for (i in 0 until wallCount) {
            val p1 = boundaryWallVertices[i]
            val p2 = boundaryWallVertices[(i + 1) % wallCount]
            val edge = p2 - p1
            // 顶点按逆时针排列时，内法线为 (-edge.y, edge.x) 归一化
            val inwardNormal = edge.perp().norm()
            // 确保法线指向 center
            val toCenter = center - p1
            val normal = if (inwardNormal.dot(toCenter) >= 0f) inwardNormal else inwardNormal * -1f

            // 查找穿透最深的顶点
            var maxPenetration = 0f
            var deepestContact: Vec2? = null

            for (v in worldVerts) {
                val signedDist = (v - p1).dot(normal)
                if (signedDist < 0f) {
                    val pen = -signedDist
                    if (pen > maxPenetration) {
                        maxPenetration = pen
                        deepestContact = v
                    }
                }
            }

            if (maxPenetration > 0f && deepestContact != null) {
                // 位置修正
                b.pos += normal * maxPenetration

                // 冲量反弹（考虑角动量）
                val r = deepestContact - b.pos
                val velAtContact = b.vel + Vec2(-b.angularVel * r.y, b.angularVel * r.x)
                val velAlongNormal = velAtContact.dot(normal)

                if (velAlongNormal < 0f) {
                    val rCrossN = r.cross(normal)
                    val invMassSum = b.invMass + rCrossN * rCrossN * b.invInertia
                    val j = -(1f + restitution) * velAlongNormal / invMassSum

                    val impulse = normal * j
                    b.vel += impulse * b.invMass
                    b.angularVel += r.cross(impulse) * b.invInertia
                }
            }
        }
    }

    // ==================== 凸多边形 SAT 碰撞检测与求解 ====================
    private fun resolveBodyPairCollision(a: RigidBody, b: RigidBody) {
        // 包围圆快速剔除
        val distSq = (b.pos - a.pos).lenSq()
        val rSum = a.boundingRadius + b.boundingRadius
        if (distSq > rSum * rSum) return

        val vertsA = a.getWorldVertices()
        val vertsB = b.getWorldVertices()

        var minOverlap = Float.MAX_VALUE
        var bestAxis = Vec2(0f, 0f)

        // 收集 A 和 B 的所有边法线作为分离轴
        val axes = mutableListOf<Vec2>()
        for (i in vertsA.indices) {
            val edge = vertsA[(i + 1) % vertsA.size] - vertsA[i]
            axes.add(edge.perp().norm())
        }
        for (i in vertsB.indices) {
            val edge = vertsB[(i + 1) % vertsB.size] - vertsB[i]
            axes.add(edge.perp().norm())
        }

        for (axis in axes) {
            val (minA, maxA) = projectPolygon(vertsA, axis)
            val (minB, maxB) = projectPolygon(vertsB, axis)

            val overlap = min(maxA, maxB) - max(minA, minB)
            if (overlap <= 0f) {
                return // 存在分离轴，未发生碰撞
            }
            if (overlap < minOverlap) {
                minOverlap = overlap
                bestAxis = axis
            }
        }

        // 确保法线方向从 A 指向 B
        val dirAB = b.pos - a.pos
        val normal = if (bestAxis.dot(dirAB) >= 0f) bestAxis else bestAxis * -1f

        // 位置分离（按逆质量分配）
        val totalInvMass = a.invMass + b.invMass
        if (totalInvMass <= 0f) return
        val correction = normal * (minOverlap / totalInvMass)
        a.pos -= correction * a.invMass
        b.pos += correction * b.invMass

        // 估算接触点：取深入对方最深的顶点均值
        val contactPoint = findContactPoint(vertsA, vertsB, normal)

        val rA = contactPoint - a.pos
        val rB = contactPoint - b.pos

        val velA = a.vel + Vec2(-a.angularVel * rA.y, a.angularVel * rA.x)
        val velB = b.vel + Vec2(-b.angularVel * rB.y, b.angularVel * rB.x)
        val relVel = velB - velA

        val velAlongNormal = relVel.dot(normal)
        if (velAlongNormal > 0f) return // 正在分离

        val rACrossN = rA.cross(normal)
        val rBCrossN = rB.cross(normal)
        val denom = totalInvMass + rACrossN * rACrossN * a.invInertia + rBCrossN * rBCrossN * b.invInertia

        val j = -(1f + restitution) * velAlongNormal / denom
        val impulse = normal * j

        a.vel -= impulse * a.invMass
        a.angularVel -= rA.cross(impulse) * a.invInertia

        b.vel += impulse * b.invMass
        b.angularVel += rB.cross(impulse) * b.invInertia
    }

    private fun projectPolygon(verts: List<Vec2>, axis: Vec2): Pair<Float, Float> {
        var minP = Float.MAX_VALUE
        var maxP = -Float.MAX_VALUE
        for (v in verts) {
            val p = v.dot(axis)
            if (p < minP) minP = p
            if (p > maxP) maxP = p
        }
        return Pair(minP, maxP)
    }

    private fun findContactPoint(vertsA: List<Vec2>, vertsB: List<Vec2>, normal: Vec2): Vec2 {
        // 找 A 上沿 normal 方向最靠前的顶点，与 B 上沿 -normal 方向最靠前的顶点
        var bestA = vertsA[0]
        var maxProjA = -Float.MAX_VALUE
        for (v in vertsA) {
            val p = v.dot(normal)
            if (p > maxProjA) {
                maxProjA = p
                bestA = v
            }
        }

        var bestB = vertsB[0]
        var minProjB = Float.MAX_VALUE
        for (v in vertsB) {
            val p = v.dot(normal)
            if (p < minProjB) {
                minProjB = p
                bestB = v
            }
        }
        return (bestA + bestB) * 0.5f
    }

    // ==================== 画面绘制 ====================
    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)

        // 绘制外框
        if (boundaryWallVertices.size >= 3) {
            val path = Path()
            path.moveTo(boundaryWallVertices[0].x, boundaryWallVertices[0].y)
            for (i in 1 until boundaryWallVertices.size) {
                path.lineTo(boundaryWallVertices[i].x, boundaryWallVertices[i].y)
            }
            path.close()
            canvas.drawPath(path, boundaryBgPaint)
            canvas.drawPath(path, boundaryPaint)
        }

        // 绘制每个多边形刚体
        for (b in bodies) {
            val verts = b.getWorldVertices()
            if (verts.size < 3) continue

            val path = Path()
            path.moveTo(verts[0].x, verts[0].y)
            for (i in 1 until verts.size) {
                path.lineTo(verts[i].x, verts[i].y)
            }
            path.close()

            bodyFillPaint.color = b.color
            canvas.drawPath(path, bodyFillPaint)
            canvas.drawPath(path, bodyStrokePaint)

            // 绘制中心到第一个顶点的旋转参考线，直观展示角速度变化
            canvas.drawLine(b.pos.x, b.pos.y, verts[0].x, verts[0].y, markerPaint)
        }
    }
}

// ==================== 主界面与实时参数控制面板 ====================
class MainActivity : AppCompatActivity() {

    private lateinit var simView: PhysicsSimulationView
    private var selectedBodyIndex = 0

    private val colorPalette = listOf(
        "红" to Color.parseColor("#F43F5E"),
        "绿" to Color.parseColor("#10B981"),
        "橙" to Color.parseColor("#F59E0B"),
        "紫" to Color.parseColor("#8B5CF6"),
        "蓝" to Color.parseColor("#3B82F6"),
        "青" to Color.parseColor("#06B6D4")
    )

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(Color.parseColor("#020617"))
        }

        // 上半部分：物理画布
        simView = PhysicsSimulationView(this)
        val simParams = LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT,
            0,
            1f
        )
        root.addView(simView, simParams)

        // 下半部分：可滚动参数控制区
        val scroll = ScrollView(this).apply {
            setBackgroundColor(Color.parseColor("#1E293B"))
        }
        val scrollParams = LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT,
            dp(320)
        )

        val panel = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(dp(14), dp(10), dp(14), dp(14))
        }

        // 1. 数量与外框选择行
        val rowGlobal = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }

        val countSpinner = Spinner(this).apply {
            adapter = spinnerAdapter(listOf("2个物体", "3个物体", "4个物体"))
            minimumHeight = dp(50)
            setSelection(1) // 默认3个
        }

        val boundarySpinner = Spinner(this).apply {
            adapter = spinnerAdapter(BoundaryType.values().map { it.label })
            minimumHeight = dp(50)
        }

        val resetBtn = Button(this).apply {
            text = "重新发射"
            textSize = 16f
            setTextColor(Color.WHITE)
            backgroundTintList = android.content.res.ColorStateList.valueOf(Color.parseColor("#0F766E"))
            minHeight = dp(48)
        }

        rowGlobal.addView(countSpinner, LinearLayout.LayoutParams(0, dp(50), 1f))
        rowGlobal.addView(boundarySpinner, LinearLayout.LayoutParams(0, dp(50), 1.2f))
        panel.addView(rowGlobal)

        // 重新发射按钮独占一行，避免挤压上方两个选择框
        val resetRow = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER
            setPadding(0, dp(2), 0, dp(4))
        }
        resetRow.addView(resetBtn, LinearLayout.LayoutParams(
            ViewGroup.LayoutParams.MATCH_PARENT, dp(48)
        ))
        panel.addView(resetRow)

        // 2. 选择要编辑的多边形对象
        val targetSpinner = Spinner(this)
        fun refreshTargetSpinner() {
            val items = simView.bodies.indices.map { "编辑对象 #${it + 1}" }
            targetSpinner.adapter = spinnerAdapter(items)
            targetSpinner.minimumHeight = dp(50)
            selectedBodyIndex = selectedBodyIndex.coerceIn(0, simView.bodies.lastIndex)
            targetSpinner.setSelection(selectedBodyIndex)
        }
        refreshTargetSpinner()
        panel.addView(targetSpinner)

        // 3. 形状与填充颜色行
        val rowShapeColor = LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }

        val shapeSpinner = Spinner(this).apply {
            adapter = spinnerAdapter(BodyShapeType.values().map { it.label })
            minimumHeight = dp(50)
        }

        val colorSpinner = Spinner(this).apply {
            adapter = spinnerAdapter(colorPalette.map { "颜色: ${it.first}" })
            minimumHeight = dp(50)
        }

        rowShapeColor.addView(shapeSpinner, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1.2f))
        rowShapeColor.addView(colorSpinner, LinearLayout.LayoutParams(0, ViewGroup.LayoutParams.WRAP_CONTENT, 1f))
        panel.addView(rowShapeColor)

        // 4. 边数调节滑块
        val sidesLabel = TextView(this).apply {
            setTextColor(Color.WHITE)
            textSize = 16f
            text = "边数: 3"
            setPadding(0, dp(5), 0, dp(2))
        }
        val sidesSeek = SeekBar(this).apply {
            max = 9 // 3..12
            progress = 0
        }
        panel.addView(sidesLabel)
        panel.addView(sidesSeek)

        // 5. 边长/尺寸调节滑块
        val sizeLabel = TextView(this).apply {
            setTextColor(Color.WHITE)
            textSize = 16f
            text = "边长/半径: 75"
            setPadding(0, dp(5), 0, dp(2))
        }
        val sizeSeek = SeekBar(this).apply {
            max = 100 // 35..135
            progress = 40
        }
        panel.addView(sizeLabel)
        panel.addView(sizeSeek)

        // 同步控件显示到当前选中物体
        var isSyncingUI = false
        fun syncControlsFromSelectedBody() {
            val b = simView.bodies.getOrNull(selectedBodyIndex) ?: return
            isSyncingUI = true
            shapeSpinner.setSelection(BodyShapeType.values().indexOf(b.shapeType))
            val colorIdx = colorPalette.indexOfFirst { it.second == b.color }.coerceAtLeast(0)
            colorSpinner.setSelection(colorIdx)
            sidesSeek.progress = (b.sides - 3).coerceIn(0, 9)
            sidesLabel.text = "边数（多边形有效）: ${b.sides}"
            sizeSeek.progress = (b.sideLength.toInt() - 35).coerceIn(0, 100)
            sizeLabel.text = "边长/尺寸: ${b.sideLength.toInt()} px"
            isSyncingUI = false
        }
        syncControlsFromSelectedBody()

        // 绑定事件监听
        countSpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(p: AdapterView<*>?, v: View?, pos: Int, id: Long) {
                val targetCount = pos + 2
                if (simView.bodies.size != targetCount) {
                    simView.initDefaultBodies(targetCount)
                    simView.repositionBodiesSafely()
                    selectedBodyIndex = 0
                    refreshTargetSpinner()
                    syncControlsFromSelectedBody()
                }
            }
            override fun onNothingSelected(p: AdapterView<*>?) {}
        }

        boundarySpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(p: AdapterView<*>?, v: View?, pos: Int, id: Long) {
                simView.setBoundary(BoundaryType.values()[pos])
            }
            override fun onNothingSelected(p: AdapterView<*>?) {}
        }

        resetBtn.setOnClickListener {
            simView.initDefaultBodies(simView.bodies.size)
            simView.repositionBodiesSafely()
            refreshTargetSpinner()
            syncControlsFromSelectedBody()
        }

        targetSpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(p: AdapterView<*>?, v: View?, pos: Int, id: Long) {
                selectedBodyIndex = pos
                syncControlsFromSelectedBody()
            }
            override fun onNothingSelected(p: AdapterView<*>?) {}
        }

        shapeSpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(p: AdapterView<*>?, v: View?, pos: Int, id: Long) {
                if (isSyncingUI) return
                val b = simView.bodies.getOrNull(selectedBodyIndex) ?: return
                b.shapeType = BodyShapeType.values()[pos]
                b.rebuildGeometry()
            }
            override fun onNothingSelected(p: AdapterView<*>?) {}
        }

        colorSpinner.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(p: AdapterView<*>?, v: View?, pos: Int, id: Long) {
                if (isSyncingUI) return
                val b = simView.bodies.getOrNull(selectedBodyIndex) ?: return
                b.color = colorPalette[pos].second
            }
            override fun onNothingSelected(p: AdapterView<*>?) {}
        }

        sidesSeek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(sb: SeekBar?, progress: Int, fromUser: Boolean) {
                if (isSyncingUI) return
                val b = simView.bodies.getOrNull(selectedBodyIndex) ?: return
                b.sides = progress + 3
                sidesLabel.text = "边数（多边形有效）: ${b.sides}"
                b.rebuildGeometry()
            }
            override fun onStartTrackingTouch(sb: SeekBar?) {}
            override fun onStopTrackingTouch(sb: SeekBar?) {}
        })

        sizeSeek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(sb: SeekBar?, progress: Int, fromUser: Boolean) {
                if (isSyncingUI) return
                val b = simView.bodies.getOrNull(selectedBodyIndex) ?: return
                b.sideLength = (progress + 35).toFloat()
                sizeLabel.text = "边长/尺寸: ${b.sideLength.toInt()} px"
                b.rebuildGeometry()
            }
            override fun onStartTrackingTouch(sb: SeekBar?) {}
            override fun onStopTrackingTouch(sb: SeekBar?) {}
        })

        scroll.addView(panel)
        root.addView(scroll, scrollParams)
        setContentView(root)
    }


    private fun spinnerAdapter(items: List<String>): ArrayAdapter<String> {
        return object : ArrayAdapter<String>(this, android.R.layout.simple_spinner_item, items) {
            override fun getView(position: Int, convertView: View?, parent: ViewGroup): View {
                val view = super.getView(position, convertView, parent) as TextView
                view.setTextColor(Color.parseColor("#F8FAFC"))
                view.textSize = 16f
                view.setPadding(dp(8), dp(8), dp(8), dp(8))
                return view
            }
            override fun getDropDownView(position: Int, convertView: View?, parent: ViewGroup): View {
                val view = super.getDropDownView(position, convertView, parent) as TextView
                view.setTextColor(Color.parseColor("#F8FAFC"))
                view.setBackgroundColor(Color.parseColor("#334155"))
                view.textSize = 16f
                view.setPadding(dp(14), dp(12), dp(14), dp(12))
                return view
            }
        }
    }

    private fun dp(value: Int): Int = (value * resources.displayMetrics.density).toInt()
}
